# -*- coding: utf-8 -*-
"""
road_edge_detector.detector

道路端（歩道と車道の境界）を画像から検知する処理の本体。
ROS に依存しないので、ROS ノード (scripts/road_edge_detector_node.py) と
評価スクリプト (scripts/evaluate_bag.py) の両方から同じ処理を呼び出せる。

処理の流れ
  1. 画像を process_width まで縮小し、ROI（台形）を長方形に射影変換する
  2. 明るさ(L)チャンネルに CLAHE とぼかしをかけ、LSD で線分を抽出する
  3. ほぼ横向きの線分のうち、端が近く向きがそろったものをつないで「長い線」にする
     （高さの違う線はつながない。広角の歪みで曲がった線は 2 次曲線で表す）
  4. 横方向に十分長い線を境界の候補にする
  5. 候補の上側・下側に帯を取り、明るさ・色・模様の差（段差の強さ）を区間ごとに測る
     - 線から少し離れた帯でも差が残ることを条件にして、細い白線などを除外する
     - 境界の向こう側の方が「今いる路面（ROI の下端）」と違うことを確かめる
  6. 横断歩道の白線を探す（v0.2）。明るい帯（上の縁と下の縁の組）が同じ明るさで 2 本以上並んでいるか、
     両側が同じ路面の、とても明るい帯が 1 本あれば「横断歩道」とみなす
     - 横断歩道があれば、いちばん手前の白線より手前で、白線にいちばん近い境界を道路端とする
       （道路端が弱くても、白線や向こう側の道路端と間違えないため。条件は通常より緩い）
     - 横断歩道がなければ、条件を満たす候補のうち、十分に強いもので一番手前（ロボット側）のものを道路端とする
  7. 画像中央の列で、その線が写っている縦位置を pixel_y / normalized_y とする
  8. 連続検出（EdgeTracker）で、数フレームそろった位置だけを「確定」として配信する（v0.2 で改良）
"""
import math
import struct
import warnings

import cv2
import numpy as np
from pylsd.lsd import lsd

# --------------------------------------------------------------------------
# パラメータの既定値（config/c2.yaml と同じ値）
# --------------------------------------------------------------------------
DEFAULT_PARAMS = {
    # 処理解像度
    "process_width": 720,          # 処理する画像の幅 [px]（C2 の 2880 の 1/4）

    # ROI（台形）。roi_units が normalized なら 0〜1、pixel なら入力画像のピクセル座標
    "roi_units": "normalized",
    "roi_upper_left": [0.30, 0.65],
    "roi_upper_right": [0.70, 0.65],
    "roi_lower_left": [0.00, 0.98],
    "roi_lower_right": [1.00, 0.98],

    # 線分抽出（LSD）
    "clahe_clip": 2.0,
    "clahe_grid": 4,
    "blur_ksize": 5,
    "lsd_scale": 0.6,
    "seg_min_len": 0.03,           # 線分の最小長（射影後の幅に対する割合）
    "max_angle": 35.0,             # 横向きとみなす傾き [deg]

    # 線分の連結
    "link_angle": 12.0,            # つなぐ線分同士の角度差の上限 [deg]
    "link_dist": 0.012,            # つなぐときの縦ずれの上限（射影後の高さに対する割合）
    "link_gap": 0.20,              # つなぐときの横の隙間の上限（射影後の幅に対する割合）
    "min_coverage": 0.35,          # 候補に必要な横方向の長さ（射影後の幅に対する割合）
    "max_rms": 0.015,              # 曲線への当てはまりの誤差の上限（高さに対する割合）

    # 境界の上下の見た目の差
    "band_gap": 0.012,             # 境界から帯までの隙間（高さに対する割合）
    "band_gap2": 0.035,            # 細い帯を除外するための、離れた位置の隙間
    "band_width": 0.040,           # 帯の厚さ（高さに対する割合）
    "n_bins": 8,                   # 線を何区間に分けて測るか
    "sigma_L": 18.0,               # 明るさの差の目安（Lab の L, 0〜255）
    "sigma_ab": 8.0,               # 色の差の目安（Lab の a,b）
    "sigma_T": 0.45,               # 模様の差の目安（勾配の強さの比の対数）
    "ref_height": 0.12,            # 「今いる路面」とみなす ROI 下端の帯の高さ

    # 道路端の選び方
    "step_th": 2.0,                # 区間ごとの段差の強さの閾値
    "own_th": 0.0,                 # 区間ごとの「今いる路面との差」の閾値
    "pass_ratio": 0.6,             # 両方の条件を満たす区間の割合の下限
    "rel_th": 0.3,                 # 画像内で一番強い境界に対する相対的な強さの下限
    "band_ext": 0.035,             # 選んだ線の手前この範囲（正規化 y）に有効な線があれば寄せる

    # 横断歩道の白線（v0.2）
    "cw_enable": True,             # 白線を手がかりに道路端を選ぶか
    "cw_dL": 30.0,                 # 白線の縁とみなす、線の両側の明るさ（L, 0〜255）の差
    "cw_Lmin": 120.0,              # 白線の明るさの下限
    "cw_dLsame": 20.0,             # 同じ横断歩道の白線どうしの明るさの差の上限
    "cw_hmax": 0.12,               # 白線 1 本の高さの上限（正規化 y）
    "cw_gap_ratio": 2.5,           # 白線どうしの隙間の上限（白線の高さに対する倍率）
    "cw_min_stripes": 2,           # 横断歩道とみなす白線の本数
    "cw_single": True,             # 白線が 1 本しか見えていなくても、次の条件なら横断歩道とみなす
    "cw_single_L": 145.0,          #   白線の明るさの下限
    "cw_single_dL": 50.0,          #   白線と周りの路面の明るさの差の下限
    "cw_single_same": 12.0,        #   白線の向こう側と手前側の路面の明るさの差の上限
    "cw_margin": 0.003,            # 白線の手前の縁からこれ以上手前の線だけを道路端の候補にする（正規化 y）
    "cw_search": 0.2,              # 白線の手前の縁から道路端を探す範囲（正規化 y）
    "cw_step_th": 1.0,             # 白線の手前で道路端に求める段差の強さの下限（通常の step_th より緩い）

    # 点字ブロック（v0.3.1）。線のすぐ向こう側か手前側が黄色の線は、点字ブロックの縁なので道路端にしない
    "tactile_enable": True,
    "tactile_db": 15.0,            # 黄色とみなす色（Lab の b を 128 を 0 として）の下限
    "tactile_amin": -8.0,          # 黄色とみなす色の a（128 を 0 として）の下限（緑のフェンスや草を除くため）
    "tactile_frac": 0.5,           # 線の片側の帯のうち、黄色の画素がこの割合以上なら点字ブロックの縁

    # 白い塗装の縁を除く（v0.3.2。既定は使わない。E06 のように横断歩道の白線や車線の白線が道路端のすぐ向こうにある場所で、
    # targets の detector: で有効にする）。線の片側が paint_L 以上に明るく、反対側より paint_dL 以上明るければ塗装の縁
    "paint_exclude": False,
    "paint_L": 145.0,
    "paint_dL": 50.0,

    # 道路端の傾きの範囲（v0.3.2。既定は制限なし。交差点の角のように、道路端が決まった向きに斜めに見える場所で
    # targets の detector: に書く）。元画像での傾き [deg]。正 = 右へ行くほど手前（画像の下）に下がる線
    "edge_angle_min": None,
    "edge_angle_max": None,

    # 曲がった・斜めの道路端の続き（v0.3.1。表示用。配信する位置 pixel_y は変えない）
    # 見つけた線の両端から、向きがなめらかにつながる線分（横向きでないものも含む）をたどって、線を延ばして描く
    "extend_enable": True,
    "extend_link": 0.05,           # 線の端と次の線分の端の距離の上限（射影後の幅に対する割合）
    "extend_turn": 35.0,           # 1 回に曲がってよい角度 [deg]
    "extend_max_angle": 75.0,      # たどる線分の傾きの上限（射影後の画像で、横から測った角度）[deg]
    "extend_steps": 8,             # たどる線分の数の上限（片側）
}


def merge_params(params=None):
    """既定値に上書きした辞書を返す（知らないキーは無視せず残す）"""
    p = dict(DEFAULT_PARAMS)
    if params:
        p.update(params)
    return p


# --------------------------------------------------------------------------
# 画像のデコード
# --------------------------------------------------------------------------
_REDUCED_FLAGS = {1: cv2.IMREAD_COLOR, 2: cv2.IMREAD_REDUCED_COLOR_2,
                  4: cv2.IMREAD_REDUCED_COLOR_4, 8: cv2.IMREAD_REDUCED_COLOR_8}


_SOF_MARKERS = (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF)


def _sof_size(b, i):
    """b[i] = 0xFF, b[i+1] = SOF マーカーの位置から (width, height) を読む。おかしな値なら None"""
    if i + 10 > len(b):
        return None
    seg_len = struct.unpack(">H", b[i + 2:i + 4])[0]
    precision = b[i + 4]
    h, w = struct.unpack(">HH", b[i + 5:i + 9])
    ncomp = b[i + 9]
    if precision in (8, 12, 16) and ncomp in (1, 3, 4) and seg_len == 8 + 3 * ncomp and w > 0 and h > 0:
        return w, h
    return None


def jpeg_size(data):
    """JPEG のヘッダから (width, height) を読む。読めなければ None"""
    b = bytes(data)
    n = len(b)
    if n < 4 or b[0] != 0xFF or b[1] != 0xD8:
        return None
    # 1) セグメントを順にたどる
    i = 2
    while i + 3 < n:
        if b[i] != 0xFF:
            break                       # 構造が想定と違う → 2) へ
        while i + 1 < n and b[i + 1] == 0xFF:
            i += 1                      # 埋め草の 0xFF を飛ばす
        marker = b[i + 1]
        if marker in _SOF_MARKERS:
            size = _sof_size(b, i)
            if size:
                return size
            break
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        if marker in (0xD9, 0xDA):      # 画像の終わり / 画像データの開始 → SOF が見つからなかった
            break
        i += 2 + struct.unpack(">H", b[i + 2:i + 4])[0]
    # 2) 見つからなければ、SOF マーカーのバイト列を全体から探して妥当性を確かめる
    for m in (0xC0, 0xC2, 0xC1):
        j = b.find(bytes((0xFF, m)))
        while j >= 0:
            size = _sof_size(b, j)
            if size:
                return size
            j = b.find(bytes((0xFF, m)), j + 2)
    return None


def choose_reduction(orig_width, process_width, margin=2):
    """
    デコード時に縮小できる倍率（1,2,4,8）を選ぶ。
    縮小デコード（JPEG の DCT 上での縮小）はザラつきが残り、検知結果が少し変わるため、
    処理幅の margin 倍までにとどめ、残りは検知側で面積平均（INTER_AREA）で縮小する。
    例: 2880 幅・処理幅 720 → 1/2 でデコード（1440 幅）→ 面積平均で 720 幅へ
    """
    r = 1
    for k in (2, 4, 8):
        if orig_width / k >= process_width * margin:
            r = k
    return r


def decode_compressed(data, process_width, reduction=1, orig_size=None):
    """
    圧縮画像をデコードする。戻り値: (画像, 元画像の (width, height))。失敗したら (None, None)
    reduction: 1 = 元の解像度でデコード（既定。v0.1 の評価はすべてこの方法と同じ結果）
               2 / 4 / 8 = JPEG を縮小しながらデコード（速いが、OpenCV 4.2 では効かず、版によって結果が少し変わる）
               "auto" = 処理幅の 2 倍まで縮小デコード
    """
    buf = np.frombuffer(data, np.uint8)
    if orig_size is None:
        orig_size = jpeg_size(data)
    if reduction == "auto":
        r = choose_reduction(orig_size[0], process_width) if orig_size else 1
    else:
        try:
            r = int(reduction)
        except (TypeError, ValueError):
            r = 1
    img = cv2.imdecode(buf, _REDUCED_FLAGS.get(r, cv2.IMREAD_COLOR))
    if img is None:
        return None, None
    if orig_size is None:   # ヘッダが読めない形式（PNG など）は縮小なしでデコードされている
        orig_size = (img.shape[1], img.shape[0])
    return img, orig_size


# --------------------------------------------------------------------------
# ROI と射影変換
# --------------------------------------------------------------------------
def roi_points(p, width, height):
    """ROI の4点 [左上, 右上, 左下, 右下] を、与えた画像サイズのピクセル座標で返す"""
    keys = ("roi_upper_left", "roi_upper_right", "roi_lower_left", "roi_lower_right")
    pts = []
    for k in keys:
        x, y = float(p[k][0]), float(p[k][1])
        if p.get("roi_units", "normalized") == "normalized":
            x, y = x * (width - 1), y * (height - 1)
        pts.append((x, y))
    return np.float32(pts)


def warp_roi(img, src):
    """台形 src を長方形に射影変換する（幅 = 上辺の長さ、高さ = 左辺の長さ。既存ノードと同じ決め方）"""
    dw = int(math.floor(np.linalg.norm(src[1] - src[0])))
    dh = int(math.floor(np.linalg.norm(src[2] - src[0])))
    dw, dh = max(dw, 8), max(dh, 8)
    M = cv2.getPerspectiveTransform(src, np.float32([[0, 0], [dw, 0], [0, dh], [dw, dh]]))
    return cv2.warpPerspective(img, M, (dw, dh), flags=cv2.INTER_AREA), M


# --------------------------------------------------------------------------
# 線分の抽出と連結
# --------------------------------------------------------------------------
class _UnionFind(object):
    def __init__(self, n):
        self.p = list(range(n))

    def find(self, i):
        while self.p[i] != i:
            self.p[i] = self.p[self.p[i]]
            i = self.p[i]
        return i

    def union(self, a, b):
        self.p[self.find(a)] = self.find(b)


def _y_at(seg, x):
    x1, y1, x2, y2 = seg[:4]
    if abs(x2 - x1) < 1e-6:
        return y1
    return y1 + (y2 - y1) * (x - x1) / (x2 - x1)


def extract_segments(warped, p):
    """LSD で線分を抽出し、ほぼ横向きで十分長いものだけ返す"""
    L = cv2.cvtColor(warped, cv2.COLOR_BGR2LAB)[:, :, 0]
    g = int(p["clahe_grid"])
    L = cv2.createCLAHE(float(p["clahe_clip"]), (g, g)).apply(L)
    k = int(p["blur_ksize"]) | 1
    L = cv2.GaussianBlur(L, (k, k), 0)
    raw = lsd(L.astype(np.float64), scale=float(p["lsd_scale"]))
    raw = raw if raw is not None else np.zeros((0, 5))
    dh, dw = warped.shape[:2]
    segs = []
    for row in raw:
        x1, y1, x2, y2 = row[:4]
        if x2 < x1:
            x1, y1, x2, y2 = x2, y2, x1, y1
        length = math.hypot(x2 - x1, y2 - y1)
        ang = math.degrees(math.atan2(y2 - y1, x2 - x1))
        if length >= p["seg_min_len"] * dw and abs(ang) <= p["max_angle"]:
            segs.append((x1, y1, x2, y2, length, ang))
    return raw, segs


def group_segments(segs, dw, dh, p):
    """端が近く、向きがそろい、高さのずれが小さい線分同士をつなぐ"""
    n = len(segs)
    uf = _UnionFind(n)
    for i in range(n):
        a = segs[i]
        for j in range(i + 1, n):
            b = segs[j]
            if abs(a[5] - b[5]) > p["link_angle"]:
                continue
            gap = max(a[0], b[0]) - min(a[2], b[2])        # 横の隙間（負なら重なり）
            if gap > p["link_gap"] * dw:
                continue
            if gap <= 0:     # 横に重なる → 重なりの中央で縦の差
                xc = (max(a[0], b[0]) + min(a[2], b[2])) / 2.0
                d = abs(_y_at(a, xc) - _y_at(b, xc))
            else:            # 離れている → 延長した継ぎ目で縦の差
                left, right = (a, b) if a[2] <= b[0] else (b, a)
                d = max(abs(_y_at(left, right[0]) - right[1]),
                        abs(_y_at(right, left[2]) - left[3]))
            if d <= p["link_dist"] * dh + 0.08 * max(gap, 0.0):
                uf.union(i, j)
    groups = {}
    for i in range(n):
        groups.setdefault(uf.find(i), []).append(segs[i])
    return list(groups.values())


def fit_group(group, dw):
    """つながった線分の点列に曲線を当てはめ、横方向の被覆率と誤差を求める"""
    xs, ys, ivs = [], [], []
    for x1, y1, x2, y2, length, _ in group:
        t = np.linspace(0.0, 1.0, max(2, int(length / 3)))
        xs.append(x1 + (x2 - x1) * t)
        ys.append(y1 + (y2 - y1) * t)
        ivs.append((x1, x2))
    xs = np.concatenate(xs)
    ys = np.concatenate(ys)
    ivs.sort()
    cov, (cs, ce) = 0.0, ivs[0]
    for s_, e_ in ivs[1:]:
        if s_ > ce:
            cov += ce - cs
            cs, ce = s_, e_
        else:
            ce = max(ce, e_)
    cov += ce - cs
    x0, x1 = float(xs.min()), float(xs.max())
    deg = 2 if (x1 - x0) >= 0.5 * dw and len(group) >= 3 else 1
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        coef = np.polyfit(xs, ys, deg)
    rms = float(np.sqrt(np.mean((np.polyval(coef, xs) - ys) ** 2)))
    return {"coef": coef, "x0": x0, "x1": x1, "coverage": cov / float(dw), "rms": rms}


# --------------------------------------------------------------------------
# 境界の上下の見た目の差
# --------------------------------------------------------------------------
def _dist(X, Y, p):
    """見た目の差 = 明るさの差 + 色の差 + 模様の差（それぞれ目安の値で割って足す）"""
    return (abs(X[0] - Y[0]) / p["sigma_L"]
            + math.hypot(X[1] - Y[1], X[2] - Y[2]) / p["sigma_ab"]
            + abs(math.log((X[3] + 1.0) / (Y[3] + 1.0))) / p["sigma_T"])


def _band(lab, grad, xs, yc, k0, k1, sign):
    """曲線 yc の上(sign=-1) / 下(sign=+1) に距離 k0〜k1 の帯を取り、(L, a, b, 模様) を返す"""
    H = lab.shape[0]
    ks = np.arange(k0, k1)
    yy = np.round(yc[None, :] + sign * ks[:, None]).astype(int)
    xx = np.broadcast_to(xs[None, :], yy.shape)
    m = (yy >= 0) & (yy < H)
    if m.sum() < 0.3 * m.size or m.sum() < 8:
        return None
    med = np.median(lab[yy[m], xx[m]], axis=0)
    return (float(med[0]), float(med[1]), float(med[2]), float(grad[yy[m], xx[m]].mean()))


def _side_yellow(lab, cand, dh, p):
    """線のすぐ向こう側(far) / 手前側(near) の帯のうち、黄色（点字ブロック）の画素の割合（v0.3.1）"""
    g1 = max(1, int(round(p["band_gap"] * dh)))
    w = max(2, int(round(p["band_width"] * dh)))
    xs = np.arange(int(cand["x0"]), max(int(cand["x0"]) + 1, int(cand["x1"])))
    yc = np.polyval(cand["coef"], xs)
    H = lab.shape[0]
    out = []
    for sign in (-1, +1):
        ks = np.arange(g1, g1 + w)
        yy = np.round(yc[None, :] + sign * ks[:, None]).astype(int)
        xx = np.broadcast_to(xs[None, :], yy.shape)
        m = (yy >= 0) & (yy < H)
        if m.sum() < 8:
            out.append(0.0)
            continue
        v = lab[yy[m], xx[m]]
        out.append(float(np.mean((v[:, 2] - 128.0 >= p["tactile_db"]) & (v[:, 1] - 128.0 >= p["tactile_amin"]))))
    return out[0], out[1]


def _bin_features(lab, grad, cand, dh, p, ref):
    g1 = max(1, int(round(p["band_gap"] * dh)))
    g2 = max(g1 + 1, int(round(p["band_gap2"] * dh)))
    w = max(2, int(round(p["band_width"] * dh)))
    edges = np.linspace(cand["x0"], cand["x1"], int(p["n_bins"]) + 1)
    steps, owns = [], []
    side_a, side_b = [], []
    for b0, b1 in zip(edges[:-1], edges[1:]):
        xs = np.arange(int(b0), max(int(b0) + 1, int(b1)))
        yc = np.polyval(cand["coef"], xs)
        A1 = _band(lab, grad, xs, yc, g1, g1 + w, -1)
        B1 = _band(lab, grad, xs, yc, g1, g1 + w, +1)
        A2 = _band(lab, grad, xs, yc, g2, g2 + w, -1)
        B2 = _band(lab, grad, xs, yc, g2, g2 + w, +1)
        if A1 is not None:
            side_a.append(A1[0])
        if B1 is not None:
            side_b.append(B1[0])
        if A1 is None or B1 is None or A2 is None or B2 is None:
            continue
        # 境界のすぐ近くと少し離れた位置の両方で差があること（細い白線はここで落ちる）
        steps.append(min(_dist(A1, B1, p), _dist(A2, B2, p)))
        # 向こう側(A)の方が、手前側(B)より「今いる路面」と違うこと
        owns.append(_dist(A2, ref, p) - _dist(B2, ref, p))
    # 線のすぐ向こう側・手前側の明るさ（白線を見つけるのに使う）
    cand["L_far"] = float(np.median(side_a)) if side_a else float("nan")
    cand["L_near"] = float(np.median(side_b)) if side_b else float("nan")
    return np.array(steps), np.array(owns)


# --------------------------------------------------------------------------
# 横断歩道の白線
# --------------------------------------------------------------------------
def find_bright_bands(cands, p):
    """
    明るい帯 = 上の縁（向こう側が暗く手前側が明るい線）と、そのすぐ手前の下の縁（向こう側が明るく手前側が暗い線）の組。
    戻り値: [(上の縁の y, 下の縁の y, 帯の明るさ, 周りの明るさ, 帯の向こう側の明るさ, 帯の手前側の明るさ), ...]（y は正規化）
    """
    cs = sorted(cands, key=lambda c: c["norm_y"])
    bands = []
    cs = [c for c in cs if not c.get("tactile", False)]      # 黄色い点字ブロックは白線ではない（v0.3.1）
    for i, c in enumerate(cs):
        la, lb = c.get("L_far", np.nan), c.get("L_near", np.nan)
        if not (np.isfinite(la) and np.isfinite(lb)):
            continue
        if lb - la < p["cw_dL"] or lb < p["cw_Lmin"]:
            continue
        for c2 in cs[i + 1:]:
            if c2["norm_y"] - c["norm_y"] > p["cw_hmax"]:
                break
            la2, lb2 = c2.get("L_far", np.nan), c2.get("L_near", np.nan)
            if (np.isfinite(la2) and np.isfinite(lb2) and la2 - lb2 >= p["cw_dL"] and la2 >= p["cw_Lmin"]
                    and abs(la2 - lb) <= p["cw_dLsame"]):
                bands.append((c["norm_y"], c2["norm_y"], 0.5 * (lb + la2), 0.5 * (la + lb2), la, lb2))
                break
    return bands


def find_crosswalk(cands, p):
    """
    横断歩道の白線を探す。
      - 明るさがそろい、高さと隙間が白線らしく並んだ明るい帯が cw_min_stripes 本以上
      - または、両側が同じ路面で、とても明るい帯が 1 本（cw_single）
    戻り値: (いちばん手前の白線の下の縁の y, 白線のリスト) / 見つからなければ (None, [])
    """
    bands = sorted(find_bright_bands(cands, p))
    best, run = None, []

    def take(run_, best_):
        if len(run_) >= int(p["cw_min_stripes"]) and (best_ is None or run_[-1][1] > best_[-1][1]):
            return run_
        return best_

    for b in bands:
        if run:
            q = run[-1]
            h0, h1 = q[1] - q[0], b[1] - b[0]
            gap = b[0] - q[1]
            if (0.4 <= h1 / max(h0, 1e-4) <= 2.5 and -0.002 <= gap <= p["cw_gap_ratio"] * max(h0, h1)
                    and abs(b[2] - q[2]) <= p["cw_dLsame"]):
                run.append(b)
                continue
            best = take(run, best)
        run = [b]
    best = take(run, best)
    if best is None and p.get("cw_single", False):
        singles = [b for b in bands if b[2] >= p["cw_single_L"] and b[2] - b[3] >= p["cw_single_dL"]
                   and abs(b[4] - b[5]) <= p["cw_single_same"]]
        if singles:
            b = max(singles, key=lambda b: b[1])
            return b[1], [b]
    if best is None:
        return None, []
    return best[-1][1], best


# --------------------------------------------------------------------------
# 曲がった・斜めの道路端の続き（v0.3.1、表示用）
# --------------------------------------------------------------------------
def _seg_yellow(lab, a, b, dh, p):
    """線分 a-b の両側の帯に黄色（点字ブロック）が多いか"""
    g1 = max(1.0, p["band_gap"] * dh)
    w = max(2.0, p["band_width"] * dh)
    v = b - a
    L = float(np.hypot(v[0], v[1]))
    if L < 1e-6:
        return False
    nrm = np.float32([-v[1] / L, v[0] / L])
    ts = np.linspace(0.0, 1.0, max(4, int(L / 3)))
    base = a[None, :] + ts[:, None] * v[None, :]
    H, W = lab.shape[:2]
    for sign in (-1.0, 1.0):
        q = base + sign * (g1 + 0.5 * w) * nrm[None, :]
        xi = np.clip(np.round(q[:, 0]).astype(int), 0, W - 1)
        yi = np.clip(np.round(q[:, 1]).astype(int), 0, H - 1)
        v = lab[yi, xi]
        if np.mean((v[:, 2] - 128.0 >= p["tactile_db"]) & (v[:, 1] - 128.0 >= p["tactile_amin"])) >= p["tactile_frac"]:
            return True
    return False


def extend_curve(cand, raw, lab, dw, dh, p):
    """
    候補の線（射影後の座標の 2 次曲線）の両端から、なめらかにつながる線分をたどって延ばした点列を返す（射影後の座標）。
    道路端が交差点の角で手前に曲がってくる部分や、斜めの部分を描くため（横向きの判定条件は満たさなくてよい）
    戻り値: (点列, 延ばした線分の数)
    """
    xs = np.linspace(cand["x0"], cand["x1"], 60)
    mid = np.stack([xs, np.polyval(cand["coef"], xs)], 1).astype(np.float32)
    raw = np.asarray(raw, np.float32).reshape(-1, raw.shape[1] if len(raw) else 5) if len(raw) else np.zeros((0, 5), np.float32)
    if len(raw) == 0:
        return mid, 0
    A0, B0 = raw[:, 0:2], raw[:, 2:4]
    keep = np.hypot(*(B0 - A0).T) >= 0.5 * p["seg_min_len"] * dw
    # 両向きを並べる: k 番目と k + n 番目は同じ線分
    A = np.vstack([A0[keep], B0[keep]])
    B = np.vstack([B0[keep], A0[keep]])
    n = int(keep.sum())
    if n == 0:
        return mid, 0
    V = B - A
    L = np.hypot(V[:, 0], V[:, 1]) + 1e-9
    U = V / L[:, None]
    ang = np.degrees(np.arctan2(np.abs(U[:, 1]), np.abs(U[:, 0]) + 1e-9))
    ok_ang = ang <= p["extend_max_angle"]
    der = np.polyder(cand["coef"])
    r = p["extend_link"] * dw
    cos_turn = math.cos(math.radians(p["extend_turn"]))
    used = np.zeros(n, bool)
    ends = []
    for x_end, sgn in ((cand["x0"], -1.0), (cand["x1"], 1.0)):
        P = np.float32([x_end, np.polyval(cand["coef"], x_end)])
        d = np.float32([1.0, np.polyval(der, x_end)]) * sgn
        d /= np.hypot(*d)
        ext = []
        for _ in range(int(p["extend_steps"])):
            gap = np.hypot(*(A - P).T)
            adv = (B - P) @ d
            m = (gap <= r) & ok_ang & (U @ d >= cos_turn) & (adv > 0.5 * L) & ~np.concatenate([used, used])
            if not m.any():
                break
            score = np.where(m, adv - 2.0 * gap, -np.inf)
            k = int(np.argmax(score))
            if _seg_yellow(lab, A[k], B[k], dh, p):
                break
            used[k % n] = True
            ext.append(B[k].copy())
            P = B[k].copy()
            d = d + U[k]
            d /= max(1e-6, float(np.hypot(*d)))
            if not (0 <= P[0] < dw and 0 <= P[1] < dh):
                break
        ends.append(ext)
    left = np.float32(ends[0][::-1]).reshape(-1, 2)
    right = np.float32(ends[1]).reshape(-1, 2)
    return np.vstack([left, mid, right]), len(ends[0]) + len(ends[1])


# --------------------------------------------------------------------------
# 検知器
# --------------------------------------------------------------------------
class RoadEdgeDetector(object):
    """1 枚の画像から道路端を探す。連続検出の管理は EdgeTracker が行う"""

    def __init__(self, params=None):
        self.p = merge_params(params)

    def detect(self, img, orig_size=None):
        """
        img: BGR 画像（任意の解像度）
        orig_size: 元画像の (width, height)。縮小してデコードした画像を渡すときに与える。
                   roi_units が pixel のとき、ROI を元画像のピクセル座標として解釈するために使う
        戻り値の辞書:
          found         : 道路端が見つかったか
          pixel_y       : 画像中央の列での道路端の縦位置 [px]（入力画像の座標）
          normalized_y  : pixel_y / 入力画像の高さ
          edge_pts      : 道路端の線（入力画像の座標の点列）
          その他        : デバッグ表示用の途中結果
        """
        p = self.p
        H0, W0 = img.shape[:2]
        s = float(p["process_width"]) / W0
        if abs(s - 1.0) > 1e-6:
            small = cv2.resize(img, (int(p["process_width"]), int(round(H0 * s))),
                               interpolation=cv2.INTER_AREA)
        else:
            small = img
        h, w = small.shape[:2]
        if p.get("roi_units", "normalized") == "normalized":
            src = roi_points(p, w, h)
        else:   # ピクセル指定は元画像の座標 → 処理する画像の座標へ換算
            ow, oh = orig_size if orig_size else (W0, H0)
            src = roi_points(p, ow, oh) * np.float32([w / float(ow), h / float(oh)])
        warped, M = warp_roi(small, src)
        dh, dw = warped.shape[:2]
        Minv = np.linalg.inv(M)

        raw, segs = extract_segments(warped, p)
        groups = group_segments(segs, dw, dh, p)

        lab = cv2.cvtColor(warped, cv2.COLOR_BGR2LAB).astype(np.float32)
        Lb = cv2.GaussianBlur(lab[:, :, 0], (3, 3), 0)
        grad = cv2.magnitude(cv2.Sobel(Lb, cv2.CV_32F, 1, 0), cv2.Sobel(Lb, cv2.CV_32F, 0, 1))
        r0 = int(dh * (1.0 - p["ref_height"]))
        rmed = np.median(lab[r0:dh - 1].reshape(-1, 3), axis=0)
        ref = (float(rmed[0]), float(rmed[1]), float(rmed[2]), float(grad[r0:dh - 1].mean()))

        cands = []
        for grp in groups:
            c = fit_group(grp, dw)
            if c["coverage"] < p["min_coverage"] or c["rms"] > p["max_rms"] * dh:
                continue
            c["steps"], c["owns"] = _bin_features(lab, grad, c, dh, p, ref)
            if len(c["steps"]) == 0:
                continue
            c["step"] = float(np.median(c["steps"]))
            passed = (c["steps"] >= p["step_th"]) & (c["owns"] >= p["own_th"])
            c["pass"] = float(passed.mean())
            c["ok"] = c["pass"] >= p["pass_ratio"]
            if p.get("tactile_enable", True):
                c["yel_far"], c["yel_near"] = _side_yellow(lab, c, dh, p)
                c["tactile"] = max(c["yel_far"], c["yel_near"]) >= p["tactile_frac"]
            else:
                c["yel_far"] = c["yel_near"] = 0.0
                c["tactile"] = False
            c["paint"] = False
            if p.get("paint_exclude", False):      # 白い塗装（横断歩道・車線の白線）の縁（v0.3.2）
                la, lb = c.get("L_far", np.nan), c.get("L_near", np.nan)
                if np.isfinite(la) and np.isfinite(lb):
                    c["paint"] = bool((la >= p["paint_L"] and la - lb >= p["paint_dL"]) or
                                      (lb >= p["paint_L"] and lb - la >= p["paint_dL"]))
            # 元画像の座標に戻した線と、画像中央の列での縦位置
            xs = np.linspace(c["x0"], c["x1"], 60)
            ys = np.polyval(c["coef"], xs)
            pts = cv2.perspectiveTransform(np.float32(np.stack([xs, ys], 1))[None], Minv)[0] / s
            order = np.argsort(pts[:, 0])
            u, v = pts[order, 0], pts[order, 1]
            uc = W0 / 2.0
            c["reach_center"] = bool(u[0] <= uc <= u[-1])
            if c["reach_center"]:
                c["pixel_y"] = float(np.interp(uc, u, v))
            else:   # 中央の列まで届いていなければ、近い方の端の点で代用
                c["pixel_y"] = float(v[0] if uc < u[0] else v[-1])
            c["norm_y"] = c["pixel_y"] / H0
            c["pts"] = pts
            # 元画像での傾き（v0.3.2）。正 = 右へ行くほど手前に下がる
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                c["angle"] = float(math.degrees(math.atan(np.polyfit(u, v, 1)[0]))) if len(u) >= 2 else 0.0
            amin, amax = p.get("edge_angle_min"), p.get("edge_angle_max")
            c["slope_ok"] = not ((amin is not None and c["angle"] < float(amin)) or
                                 (amax is not None and c["angle"] > float(amax)))
            cands.append(c)

        edge, cw_y, stripes = self._select(cands)
        if p.get("extend_enable", False):   # 表示用: 曲がった・斜めの続きまで延ばした線（pixel_y は変えない）
            for c in cands:
                if c["ok"] and not c["tactile"] and not c["paint"]:
                    q, n_ext = extend_curve(c, raw, lab, dw, dh, p)
                    if n_ext:
                        c["pts_ext"] = cv2.perspectiveTransform(np.float32(q)[None], Minv)[0] / s
        # 連続検出が使う「支え」の候補（白線より向こうの線と、点字ブロックの縁は除く）
        # 3 番目の値は cands の中の番号（デバッグ画像で、配信している線を描くために使う）
        support = [(c["norm_y"], c["step"], i) for i, c in enumerate(cands)
                   if (cw_y is None or c["norm_y"] > cw_y + p["cw_margin"]) and not c["tactile"] and not c["paint"]
                   and c["slope_ok"]]
        return {
            "found": edge is not None,
            "pixel_y": edge["pixel_y"] if edge is not None else None,
            "normalized_y": edge["norm_y"] if edge is not None else None,
            "edge_pts": edge.get("pts_ext", edge["pts"]) if edge is not None else None,
            "edge": edge, "cands": cands, "raw": raw, "segs": segs,
            "warped": warped, "M": M, "scale": s, "roi_src": src / s,
            "image_size": (W0, H0),
            "crosswalk_y": cw_y,          # いちばん手前の白線の下の縁（正規化 y）。白線が無ければ None
            "stripes": stripes,           # 見つけた白線 [(上の縁の y, 下の縁の y, ...)]
            "trusted": cw_y is not None and edge is not None,   # 白線の手前の線として選んだか
            "support": support,
        }

    def _select(self, cands):
        """戻り値: (選んだ候補 または None, いちばん手前の白線の下の縁の y または None, 白線のリスト)"""
        p = self.p
        if p.get("cw_enable", True):
            cw_y, stripes = find_crosswalk(cands, p)
            if cw_y is not None:
                return self._select_before_crosswalk(cands, cw_y, stripes), cw_y, stripes
        ok = [c for c in cands if c["ok"] and not c.get("tactile", False) and not c.get("paint", False)
              and c.get("slope_ok", True)]
        if not ok:
            return None, None, []
        strongest = max(c["step"] for c in ok)
        strong = [c for c in ok if c["step"] >= p["rel_th"] * strongest]
        edge = max(strong, key=lambda c: c["norm_y"])          # 十分強いもののうち一番手前
        near = [c for c in ok if edge["norm_y"] < c["norm_y"] <= edge["norm_y"] + p["band_ext"]]
        if near:                                               # 道路端の帯のロボット側の線に寄せる
            edge = max(near, key=lambda c: c["norm_y"])
        return edge, None, []

    def _select_before_crosswalk(self, cands, cw_y, stripes):
        """
        横断歩道があるとき: いちばん手前の白線より手前（cw_margin〜cw_search）で、白線にいちばん近い境界。
        さらに手前の白線の上の縁（向こう側が暗く手前側が白線の明るさの線）は除く。段差の条件は cw_step_th に緩める
        """
        p = self.p
        paint = float(np.median([b[2] for b in stripes]))
        below = []
        for c in cands:
            y = c["norm_y"]
            if (not (cw_y + p["cw_margin"] < y <= cw_y + p["cw_search"]) or c.get("tactile", False)
                    or c.get("paint", False) or not c.get("slope_ok", True)):
                continue
            la, lb = c.get("L_far", np.nan), c.get("L_near", np.nan)
            if np.isfinite(la) and np.isfinite(lb) and lb - la >= p["cw_dL"] and abs(lb - paint) <= p["cw_dLsame"]:
                continue                                       # 次の白線の上の縁
            if len(c["steps"]) and c["step"] >= p["cw_step_th"]:
                below.append(c)
        if not below:
            return None
        return min(below, key=lambda c: c["norm_y"])


# --------------------------------------------------------------------------
# 連続検出
# --------------------------------------------------------------------------
class EdgeTracker(object):
    """
    連続検出（v0.2）
      - 位置のそろった検知が count_th フレーム続いたら「確定」
      - 確定後は、予測位置の ±gate に弱い候補（段差の強さ >= support_step）があれば、
        そのフレームの選択が別の線でも追跡を続ける（道路端が一瞬弱く写っても途切れない）
      - 確定した道路端より far_switch 以上向こう（画像の上）の線には乗り換えない。
        白線や向こう側の道路端は、近づいている道路端より必ず向こうにあるため。
        ただし、白線の手前の線として選ばれた検知（trusted）は例外
      - 予測位置と違う検知が同じ位置で count_th フレームそろったら、そちらに乗り換える
        （支えの候補がある間は、trusted の検知でないと乗り換えない）
      - max_miss フレームまでの途切れは直前の位置を保つ（hold）
      - 追っている線が画像の下（予測位置 >= exit_y）まで来たら、exit_band 以内の向こうの線への検知を
        同じ道路端の帯の車道側の線として引き継ぐ（v0.2.1。帯の歩道側の線が ROI の下に出ていくときに
        見失わないため）
      - 確定前でも、予測位置の ±pending_gate に段差の強さ >= pending_step の候補があれば、数は増やさずに
        追跡を保つ（v0.2.1。遠くの道路端は 1 フレームおきにしか選ばれないことが多いため）。
        続けて保つのは pending_max フレームまで（その間に別の線ばかり選ばれる場合に、そちらの確定を妨げないため）
      - 予測位置から外れた検知でも、直前の位置から last_gate 以内なら同じ線として更新する（v0.2.2。幅の広い段差のように
        近い 2 本の線を交互に選ぶと、速さの推定がずれて見失うことがあったため）
      - 弱い候補での更新では、下向きの速さを変えない（v0.2.2）
      - 弱い候補で、今の位置より support_up 以上向こう（画像の上）へは動かさない（v0.3.1。地面の線は、前に進む
        ロボットから遠ざからないため。横断歩道の白線の縁に乗り移るのを防ぐ）
      - 追っている線ごとに番号（track_id）と、最初に見つけた位置（y0）を持つ（v0.3.1。TargetGate が使う）
    """

    def __init__(self, count_th=3, max_jump=0.06, max_miss=2, gate=0.03, support_step=1.0, far_switch=0.03,
                 exit_y=0.93, exit_band=0.12, exit_vscale=0.7, pending_gate=0.02, pending_step=2.0, pending_max=5,
                 last_gate=0.02, support_up=0.012):
        self.count_th = max(1, int(count_th))
        self.max_jump = float(max_jump)
        self.max_miss = max(0, int(max_miss))
        self.gate = float(gate)
        self.support_step = float(support_step)
        self.far_switch = float(far_switch)
        self.exit_y = float(exit_y)
        self.exit_band = float(exit_band)
        self.exit_vscale = float(exit_vscale)
        self.pending_gate = float(pending_gate)
        self.pending_step = float(pending_step)
        self.pending_max = max(0, int(pending_max))
        self.last_gate = float(last_gate)
        self.support_up = float(support_up)
        self._clock = 0.0
        self.track_id = 0      # 追っている線の番号（新しい線を追い始めるたびに増える。reset では戻さない）
        self.reset()

    def reset(self):
        self.y = None          # 追っている道路端の位置（normalized_y）
        self.v = 0.0           # 下向きの速さ [normalized_y / s]
        self.t_last = None     # 最後に位置を更新した時刻 [s]
        self.count = 0         # 位置のそろった検知の数
        self.miss = 0          # 続けて途切れた（または外れ値だった）フレーム数
        self.alt_y = None      # 乗り換え候補の位置
        self.alt_count = 0
        self.hold = False      # このフレームで位置を更新できず、直前の位置を保っているか
        self.supported = False  # このフレームで弱い候補に支えられて追跡を続けたか
        self.handover = False   # このフレームで帯の車道側の線に引き継いだか
        self.pending_n = 0      # 確定前に弱い候補で続けて追跡を保ったフレーム数
        self.y0 = None          # 追っている線を最初に見つけた位置
        self.alt_y0 = None      # 乗り換え候補を最初に見つけた位置

    @property
    def confirmed(self):
        return self.y is not None and self.count >= self.count_th

    def _set(self, t, y, count):
        if self.t_last is not None and t > self.t_last and self.y is not None:
            v = (y - self.y) / (t - self.t_last)
            self.v = max(0.0, 0.5 * self.v + 0.5 * v)
        self.y, self.t_last, self.count, self.miss = float(y), t, count, 0

    def _switch(self, t):
        self.y, self.count, self.miss, self.t_last, self.v = self.alt_y, self.alt_count, 0, t, 0.0
        self.y0 = self.alt_y0 if self.alt_y0 is not None else self.alt_y
        self.track_id += 1
        self.alt_y, self.alt_count, self.alt_y0 = None, 0, None
        self.hold = False
        self.pending_n = 0

    def update(self, found, normalized_y, support=(), trusted=False, stamp=None):
        """
        found, normalized_y: このフレームの検知結果
        support: [(normalized_y, 段差の強さ), ...] このフレームの候補（RoadEdgeDetector.detect の "support"）
        trusted: 白線の手前の線として選んだ検知か（detect の "trusted"）
        stamp:   時刻 [s]（省略すると 0.1 秒おきとみなす）
        戻り値: (確定しているか, 確定している位置 normalized_y または None)
        """
        self._clock = float(stamp) if stamp is not None else self._clock + 0.1
        t = self._clock
        self.hold = False
        self.supported = False
        self.handover = False
        y = normalized_y
        if self.y is None:
            if found:
                self.y, self.count, self.miss, self.t_last, self.v = float(y), 1, 0, t, 0.0
                self.pending_n = 0
                self.y0 = float(y)
                self.track_id += 1
            return self.confirmed, (self.y if self.confirmed else None)
        dt = (t - self.t_last) if self.t_last is not None else 0.1
        pred = self.y + self.v * min(dt, 1.0)
        tol = self.gate if self.confirmed else self.max_jump
        if found and (abs(y - pred) <= tol or abs(y - self.y) <= self.last_gate):
            self._set(t, y, self.count + 1)
            self.alt_y, self.alt_count, self.alt_y0 = None, 0, None
            self.pending_n = 0
            return self.confirmed, self.y
        # 確定前: 予測位置の近くの弱い候補で追跡を保つ（数は増やさない。続けて pending_max フレームまで）
        if not self.confirmed and support and self.pending_n < self.pending_max:
            near = [c for c in support if abs(c[0] - pred) <= self.pending_gate and c[1] >= self.pending_step
                    and c[0] >= self.y - self.support_up]
            if near:
                self.y, self.t_last, self.miss = float(min(near, key=lambda c: abs(c[0] - pred))[0]), t, 0
                self.pending_n += 1
                self.supported = True
                return False, None
        # 画像の下まで来た線から、帯の向こう側（車道側）の線へ引き継ぐ
        if (self.confirmed and found and pred >= self.exit_y
                and pred - self.exit_band <= y <= pred - self.gate):
            self.y, self.t_last, self.count, self.miss = float(y), t, self.count + 1, 0
            self.v *= self.exit_vscale
            self.alt_y, self.alt_count, self.alt_y0 = None, 0, None
            self.handover = True
            return self.confirmed, self.y
        # 予測位置の近くの弱い候補
        sup = None
        if self.confirmed and support:
            near = [c for c in support if abs(c[0] - pred) <= self.gate and c[1] >= self.support_step
                    and c[0] >= self.y - self.support_up]
            if near:
                sup = min(near, key=lambda c: abs(c[0] - pred))[0]
        # 別の線の検知は乗り換え候補として数える
        if found and (trusted or sup is None):
            far_lim = 1.0 if trusted else max(self.gate, self.far_switch)
            if not (self.confirmed and y < pred - far_lim):
                if self.alt_y is not None and abs(y - self.alt_y) <= self.max_jump:
                    self.alt_y, self.alt_count = float(y), self.alt_count + 1
                else:
                    self.alt_y, self.alt_count, self.alt_y0 = float(y), 1, float(y)
                if self.alt_count >= self.count_th:
                    self._switch(t)
                    return self.confirmed, self.y
        if sup is not None:
            v = self.v
            self._set(t, sup, self.count + 1)
            self.v = v           # 弱い候補での更新では速さを変えない
            self.supported = True
            return self.confirmed, self.y
        self.miss += 1
        self.hold = True
        if self.miss > self.max_miss:
            if self.alt_y is not None:
                self._switch(t)
            else:
                self.reset()
        return self.confirmed, (self.y if self.confirmed else None)

# -*- coding: utf-8 -*-
"""
road_edge_detector.visualize

デバッグ画像を作る。
  左: 入力画像（縮小済み）
      緑の台形 = ROI
      赤線 = このフレーム 1 枚だけで道路端として選んだ線（配信するとは限らない）
      マゼンタの太線 = 配信している道路端の線（v0.3.1 から線全体を描く。赤と重なると、赤の周りがマゼンタの縁取りになる）
      マゼンタの短い横線（両端に縦線）= 配信している位置（画像中央の列での pixel_y）
      水色の点線 = 横断歩道の白線のいちばん手前の縁（v0.2。これより向こうの線は道路端にしない）
  右: 射影変換後の画像（既存ノードのデバッグ画像に相当）
      灰色 = LSD の線分 / 水色 = 条件を満たした候補 / オレンジ = 満たさなかった候補
      黄色 = 点字ブロックの縁として除いた候補（v0.3.1）
      赤 = 選ばれた道路端 / 数字 = 段差の強さ
  上: 状態（CONFIRMED = 連続検出で確定 / raw = このフレームだけ検知 / none = 検知なし）
"""
import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX


def published_pts(res, y, prev_pts=None, prev_y=None, tol=0.004):
    """
    配信している位置 y（normalized_y）の線の点列（入力画像の座標）を返す（v0.3.1）。
    連続検出の位置は、そのフレームの候補のどれかの位置そのもの（選ばれた線、または弱い候補）なので、
    位置が tol 以内の候補の線を使う。見つからない（途切れて直前の位置を保っている）ときは、
    前のフレームの線 prev_pts を、位置の差だけ上下にずらして使う。どちらも無ければ None
    """
    if y is None:
        return None
    best, bd = None, tol
    for c in res.get("cands", []):
        d = abs(c["norm_y"] - y)
        if d <= bd:
            best, bd = c, d
    if best is not None:
        return best.get("pts_ext", best["pts"])
    if prev_pts is not None and prev_y is not None:
        H0 = res["image_size"][1]
        return prev_pts + np.float32([0.0, (y - prev_y) * H0])
    return None


def draw_debug(img, res, status="", confirmed=False, count=0, proc_ms=None, orig_height=None,
               max_width=720, track_y=None, hold=False, supported=False, handover=False, show_raw=False,
               track_pts=None):
    """
    img: 検知に渡した画像（BGR）, res: RoadEdgeDetector.detect の戻り値
    orig_height: 元画像の高さ。与えると pixel_y を元画像のピクセル座標で表示する
    max_width: 左側の画像をこの幅まで縮小して描く（配信する画像を軽くするため）
    track_y: 連続検出で確定している位置（normalized_y）。配信している値をマゼンタで描く
    hold: このフレームで検知が途切れ、直前の位置を保っているか
    supported: 弱い線に支えられて追跡を続けているか / handover: 帯の向こう側の線に引き継いだか（v0.2.1）
    show_raw: 確定していないとき、このフレームの検知位置をマゼンタで描くか（v0.3。v0.3.1 から既定は描かない）
    track_pts: 配信している線の点列（入力画像の座標。published_pts で求める）。与えると線全体をマゼンタで描く（v0.3.1）
    """
    H, W = img.shape[:2]
    f = 1.0
    if max_width and W > max_width:
        f = float(max_width) / W
        img = cv2.resize(img, (int(round(W * f)), int(round(H * f))), interpolation=cv2.INTER_AREA)
        H, W = img.shape[:2]
    k = max(1, int(round(W / 720.0)))
    left = img.copy()
    cv2.polylines(left, [np.int32(res["roi_src"][[0, 1, 3, 2]] * f)], True, (0, 255, 0), 2 * k)
    cw_y = res.get("crosswalk_y")
    if cw_y is not None:  # 水色の点線: 横断歩道の白線のいちばん手前の縁
        y = int(round(cw_y * H))
        for x in range(0, W, 16 * k):
            cv2.line(left, (x, y), (min(W - 1, x + 8 * k), y), (255, 255, 0), 2 * k)
    show_y = track_y if (confirmed and track_y is not None) else (
        res["normalized_y"] if (res["found"] and show_raw) else None)
    if show_y is not None and confirmed and track_pts is not None:
        # マゼンタの太線: 配信している道路端の線全体（赤より先に太く描くので、重なると赤の縁取りになる）
        cv2.polylines(left, [np.int32(track_pts * f)], False, (255, 0, 255), 9 * k)
    if res["found"]:     # 赤: このフレームで見つけた道路端
        cv2.polylines(left, [np.int32(res["edge_pts"] * f)], False, (0, 0, 255), 3 * k)
    if show_y is not None:   # マゼンタの短い横線: 配信している位置（画像中央の列）
        y = int(round(show_y * H))
        cv2.line(left, (W // 2 - 40 * k, y), (W // 2 + 40 * k, y), (255, 0, 255), 3 * k)
        cv2.line(left, (W // 2 - 40 * k, y - 8 * k), (W // 2 - 40 * k, y + 8 * k), (255, 0, 255), 2 * k)
        cv2.line(left, (W // 2 + 40 * k, y - 8 * k), (W // 2 + 40 * k, y + 8 * k), (255, 0, 255), 2 * k)

    tr = res["warped"].copy()
    for row in res["raw"]:
        cv2.line(tr, (int(row[0]), int(row[1])), (int(row[2]), int(row[3])), (160, 160, 160), 1)
    for c in res["cands"]:
        xs = np.linspace(c["x0"], c["x1"], 40)
        ys = np.polyval(c["coef"], xs)
        col = (0, 230, 230) if c.get("tactile") else ((255, 255, 0) if c["ok"] else (0, 165, 255))
        cv2.polylines(tr, [np.int32(np.stack([xs, ys], 1))], False, col, 2)
        cv2.putText(tr, "%.1f" % c["step"], (int(c["x0"]) + 2, max(10, int(ys[0]) - 3)),
                    FONT, 0.4, col, 1, cv2.LINE_AA)
    if res["found"]:
        e = res["edge"]
        xs = np.linspace(e["x0"], e["x1"], 40)
        ys = np.polyval(e["coef"], xs)
        cv2.polylines(tr, [np.int32(np.stack([xs, ys], 1))], False, (0, 0, 255), 3)

    tr = cv2.resize(tr, (max(1, int(round(tr.shape[1] * H / float(tr.shape[0])))), H))
    canvas = np.hstack([left, np.full((H, 4, 3), 255, np.uint8), tr])

    if confirmed:
        if hold:
            state, col = "CONFIRMED (hold)", (0, 200, 120)
        elif handover:
            state, col = "CONFIRMED (handover)", (0, 255, 120)
        elif supported:
            state, col = "CONFIRMED (weak)", (0, 230, 160)
        else:
            state, col = "CONFIRMED", (0, 255, 0)
    elif res["found"]:
        state, col = "raw (%d)" % count, (0, 255, 255)
    else:
        state, col = "none", (0, 165, 255)
    txt = "%s  %s" % (status, state)
    if cw_y is not None:
        txt += "  crosswalk"
    if show_y is not None:
        py = show_y * (orig_height if orig_height else H)
        txt += "  y=%.3f (%.0fpx)" % (show_y, py)
    if proc_ms is not None:
        txt += "  %.0fms" % proc_ms
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 24), (0, 0, 0), -1)
    cv2.putText(canvas, txt, (6, 17), FONT, 0.55, col, 1, cv2.LINE_AA)
    return canvas

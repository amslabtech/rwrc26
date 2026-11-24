#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_setup.py  road_edge_detector を動かすのに必要なものがそろっているかを確かめる（roscore は不要）

  rosrun road_edge_detector check_setup.py                 # catkin build と source の後
  python3 scripts/check_setup.py                           # ビルド前でも可（パッケージのフォルダで）
  python3 scripts/check_setup.py --image frame.jpg         # 手元の画像（C2 の画像など）で検知してみる
  python3 scripts/check_setup.py --config my_c2.yaml       # 別の設定ファイルを確かめる

確かめること
  1. Python のライブラリ（OpenCV, NumPy, PyYAML, pylsd）
  2. ROS 1 のライブラリ（rospy, std_msgs, sensor_msgs, rostopic）
  3. 設定ファイル（判定したい道路端 targets と、道路端ごとの設定に誤りがないか）
  4. 合成した画像（2880x1860 の JPEG。歩道とアスファルトの境目を 0.80 の高さに描いたもの）で検知できるかと、処理時間
  5. （--image を付けたとき）その画像での検知結果。デバッグ画像を check_setup_debug.jpg に保存する
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(HERE)

results = []


def report(ok, what, detail=""):
    results.append(ok)
    print("[%s] %s%s" % ("OK" if ok else "NG", what, ("  " + detail) if detail else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(PKG_DIR, "config", "c2.yaml"))
    ap.add_argument("--image", default="", help="検知してみる画像（JPEG など）")
    ap.add_argument("--repeat", type=int, default=10, help="処理時間を測る回数")
    args = ap.parse_args()

    print("road_edge_detector の動作確認（%s）" % PKG_DIR)
    print("Python %s" % sys.version.split()[0])

    # 1. Python のライブラリ
    try:
        import cv2
        report(True, "OpenCV", cv2.__version__)
    except ImportError as e:
        report(False, "OpenCV", "%s → sudo apt install python3-opencv" % e)
        return finish()
    try:
        import numpy as np
        report(True, "NumPy", np.__version__)
    except ImportError as e:
        report(False, "NumPy", "%s → sudo apt install python3-numpy" % e)
        return finish()
    try:
        import yaml
        report(True, "PyYAML", getattr(yaml, "__version__", ""))
    except ImportError as e:
        report(False, "PyYAML", "%s → sudo apt install python3-yaml" % e)
        return finish()
    try:
        from pylsd.lsd import lsd  # noqa: F401
        report(True, "pylsd（LSD による線分抽出）")
    except Exception as e:
        report(False, "pylsd（LSD による線分抽出）", "%s → pip3 install ocrd-fork-pylsd==0.0.8" % e)
        return finish()

    # 2. ROS 1 のライブラリ（無くても 3〜5 は確かめられる）
    src_hint = "source /opt/ros/noetic/setup.bash をしたか"
    for mod, hint in (("rospy", src_hint), ("std_msgs.msg", src_hint), ("sensor_msgs.msg", src_hint),
                      ("placenav_viz_msgs.msg", "sudo apt install ros-noetic-rostopic")):
        try:
            __import__(mod)
            report(True, "ROS: %s" % mod)
        except Exception as e:
            report(False, "ROS: %s" % mod, "%s %s" % (e, ("→ " + hint) if hint else ""))

    # このパッケージ
    try:
        from road_edge_detector.pipeline import RoadEdgePipeline
        from road_edge_detector.detector import decode_compressed
        from road_edge_detector.visualize import draw_debug
        src = "インストール済み（catkin の devel / install）"
    except ImportError:
        sys.path.insert(0, os.path.join(PKG_DIR, "src"))
        from road_edge_detector.pipeline import RoadEdgePipeline
        from road_edge_detector.detector import decode_compressed
        from road_edge_detector.visualize import draw_debug
        src = "パッケージの src から直接"
    import road_edge_detector
    report(True, "road_edge_detector を読み込めた", "%s: %s" % (src, os.path.dirname(road_edge_detector.__file__)))

    # 3. 設定ファイル
    try:
        cfg = yaml.safe_load(open(args.config)) or {}
        pipe = RoadEdgePipeline(cfg)
        tg = cfg.get("targets", [])
        special = [t.get("name", "?") for t in tg if any(t.get(k) for k in ("detector", "tracker", "gate"))]
        report(True, "設定ファイル %s" % args.config,
               "判定したい道路端 %d か所（%s）%s / ノード番号のトピック %s" % (
                   len(tg), ", ".join("%s:%s" % (t.get("name"), t.get("node")) for t in tg),
                   ("、道路端ごとの設定: " + ", ".join(special)) if special else "",
                   cfg.get("node_topic", "/toponav/localization_diagnostics")))
    except Exception as e:
        report(False, "設定ファイル %s" % args.config, str(e))
        return finish()

    # 4. 合成画像での検知と処理時間（ノード番号では絞らない）
    rng = np.random.RandomState(0)
    H, W = 1860, 2880
    img = np.zeros((H, W, 3), np.uint8)
    yb = int(0.80 * H)
    tex = cv2.GaussianBlur(rng.randint(0, 255, (H, W)).astype(np.uint8), (0, 0), 3)
    img[:yb] = 70        # 向こう側: アスファルト（暗い）
    img[yb:] = 135       # 手前側: 歩道（明るい）
    img = np.clip(img.astype(np.int16) + ((tex.astype(np.int16) - 128) // 6)[:, :, None], 0, 255).astype(np.uint8)
    img[:int(0.45 * H)] = (200, 180, 160)   # 空
    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    data = jpg.tobytes()
    pipe = RoadEdgePipeline(cfg, use_target_gate=False)
    det = pipe.base_detector
    ms, res, orig = [], None, None
    for k in range(max(1, args.repeat)):
        t0 = time.time()
        im, orig = decode_compressed(data, det.p["process_width"], 1, orig)
        res = det.detect(im, orig)
        ms.append((time.time() - t0) * 1000.0)
    if len(ms) > 1:
        ms = ms[1:]      # 1 回目は読み込みの準備が入るので除く
    y = res["normalized_y"]
    good = bool(res["found"] and abs(y - 0.80) < 0.02)
    report(good, "合成画像（境目 0.80）での検知", "検知 %s / 1 フレームの処理（JPEG のデコード込み）平均 %.0f ms・最大 %.0f ms" % (
        ("y=%.3f" % y) if res["found"] else "なし", float(np.mean(ms)), float(np.max(ms))))
    if np.mean(ms) > 100:
        print("     ※ 平均 100 ms（C2 の 10 Hz の 1 周期）を超えています。実機ではフレームを取りこぼします")

    # 5. 手元の画像
    if args.image:
        im = cv2.imread(args.image)
        if im is None:
            report(False, "画像 %s" % args.image, "読めません")
        else:
            t0 = time.time()
            res = det.detect(im)
            dt = (time.time() - t0) * 1000.0
            canvas = draw_debug(im, res, confirmed=res["found"], track_y=res["normalized_y"],
                                track_pts=res["edge_pts"] if res["found"] else None)
            outp = os.path.abspath("check_setup_debug.jpg")
            cv2.imwrite(outp, canvas)
            report(True, "画像 %s（%dx%d）" % (args.image, im.shape[1], im.shape[0]),
                   "検知 %s / %.0f ms / デバッグ画像 %s" % (("y=%.3f" % res["normalized_y"]) if res["found"] else "なし",
                                                    dt, outp))
    return finish()


def finish():
    n_ng = sum(1 for r in results if not r)
    print("")
    print("結果: %s" % ("すべて OK" if n_ng == 0 else "NG が %d 件あります（上の → の手順で直してください）" % n_ng))
    return 0 if n_ng == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

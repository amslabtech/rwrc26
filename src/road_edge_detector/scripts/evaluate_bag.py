#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
evaluate_bag.py

rosbag の画像を 1 フレームずつすべて処理し、場面台帳 (scenes.csv) と突き合わせて評価する。
roscore もノードの起動も不要。ROS ノードと同じ検知処理・同じ連続検出の条件を使うので、
何度実行しても同じ結果になる（再生速度や処理落ちの影響を受けない）。

使い方
  rosrun road_edge_detector evaluate_bag.py \\
      --bags ~/road_edge/bags/tsukuba_stop_line/whill_footcamera_c2_0913_*.bag \\
      --scenes ~/road_edge/bags/tsukuba_stop_line/scenes.csv \\
      --out ~/road_edge/eval/run1 --video

評価の考え方（scenes.csv の label ごと）
  target   : 道路端が見える区間。検知率（連続検出で確定したフレームの割合）と、
             初回検知の時刻・normalized_y を記録する。
             end_s の直前 --tail-exclude 秒は、道路端が画像中央で ROI の下に出ていることが
             多いため、検知率の計算から除いて別に数える。
  negative : 道路端が無い区間。誤検知のフレーム数と、連続した誤検知を 1 回とした回数・時刻・画像を記録する。
  ignore   : 評価しない（判別困難な区間）。
  bag の番号は、ファイル名の末尾の数字（..._3.bag なら 3）で scenes.csv の bag 列と対応させる。

出力（--out 以下）
  summary.txt            結果のまとめ（端末にも表示）
  scenes_result.csv      scenes.csv の区間ごとの結果
  events.csv             初回検知と誤検知の一覧（時刻・位置・画像ファイル）
  <bag名>/frames.csv     フレームごとの結果
  <bag名>/timeline.png   時間と検知位置のグラフ
  <bag名>/images/        初回検知・誤検知のデバッグ画像
  <bag名>/debug.mp4      全フレームのデバッグ動画（--video のとき）
"""
import argparse
import csv
import glob
import math
import os
import re
import sys
import time

import cv2
import numpy as np
import yaml

try:
    from road_edge_detector.detector import (DEFAULT_PARAMS, EdgeTracker, RoadEdgeDetector,
                                             decode_compressed)
    from road_edge_detector.visualize import draw_debug
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from road_edge_detector.detector import (DEFAULT_PARAMS, EdgeTracker, RoadEdgeDetector,
                                             decode_compressed)
    from road_edge_detector.visualize import draw_debug

PKG_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
DEFAULT_CONFIG = os.path.join(PKG_DIR, "config", "c2.yaml")


# --------------------------------------------------------------------------
def load_config(path):
    with open(path) as f:
        cfg = yaml.safe_load(f) or {}
    det = {k: cfg[k] for k in DEFAULT_PARAMS if k in cfg}
    return cfg, det


def load_scenes(path):
    rows = []
    with open(path, newline="") as f:
        for i, r in enumerate(csv.DictReader(f)):
            try:
                b = str(r["bag"]).strip()
                rows.append(dict(idx=i, bag=int(b) if b.isdigit() else b,
                                 start=float(r["start_s"]), end=float(r["end_s"]),
                                 label=str(r["label"]).strip(),
                                 edge_type=str(r.get("edge_type", "") or "").strip(),
                                 view=str(r.get("view", "") or "").strip(),
                                 place=str(r.get("place", "") or "").strip(),
                                 note=str(r.get("note", "") or "").strip()))
            except (KeyError, ValueError) as e:
                print("  ! scenes.csv の %d 行目を読めません: %s" % (i + 2, e))
    return rows


def bag_id_of(path):
    """scenes.csv の bag 列と照合する ID。ファイル名の末尾の番号（..._3.bag → 3）、無ければファイル名（tsukuba）"""
    base = os.path.basename(path)
    m = re.search(r"_(\d+)\.bag$", base)
    return int(m.group(1)) if m else os.path.splitext(base)[0]


def find_row(rows, t):
    for r in rows:
        if r["start"] <= t <= r["end"]:
            return r
    return None


def fmt(v, f="%.3f"):
    return "" if v is None or (isinstance(v, float) and math.isnan(v)) else f % v


# --------------------------------------------------------------------------
def evaluate_bag(path, bag_id, rows, args, det_params, count_th, max_jump, max_miss, trk_extra=None):
    import rosbag
    name = os.path.splitext(os.path.basename(path))[0]
    out_dir = os.path.join(args.out, name)
    img_dir = os.path.join(out_dir, "images")
    os.makedirs(img_dir, exist_ok=True)

    detector = RoadEdgeDetector(det_params)
    tracker = EdgeTracker(count_th, max_jump, max_miss, **(trk_extra or {}))
    bag = rosbag.Bag(path, "r")
    topics = bag.get_type_and_topic_info().topics
    if args.topic not in topics:
        print("  ! %s に %s がありません。スキップします" % (name, args.topic))
        return [], [], []
    total = topics[args.topic].message_count
    t0 = bag.get_start_time()

    frames, writer, orig_size = [], None, None
    state = {}   # 区間ごとの集計
    for r in rows:
        state[r["idx"]] = dict(row=r, n=0, n_conf=0, n_raw=0, n_tail=0, n_tail_conf=0, jumps=0, prev_y=None,
                               first=None, first_raw=None, events=[], in_event=False)
    unl = dict(n=0, n_conf=0)

    print("  %s (bag %s): %d フレーム" % (name, bag_id, total))
    for k, (_, msg, t) in enumerate(bag.read_messages(topics=[args.topic])):
        rel = t.to_sec() - t0
        td = time.time()
        img, orig_size = decode_compressed(msg.data, detector.p["process_width"], 1, orig_size)
        dec_ms = (time.time() - td) * 1000.0
        if img is None:
            continue
        tp = time.time()
        res = detector.detect(img, orig_size)
        det_ms = (time.time() - tp) * 1000.0
        conf, ty = tracker.update(res["found"], res["normalized_y"], res["support"], res["trusted"],
                                  stamp=msg.header.stamp.to_sec())
        ny = ty if conf else None                                   # 配信する位置（確定位置）
        raw_ny = res["normalized_y"] if res["found"] else None      # このフレーム単体の位置
        row = find_row(rows, rel)
        label = row["label"] if row else "unlabeled"
        tail = bool(row and label == "target" and rel > row["end"] - args.tail_exclude)

        canvas = None

        def debug_image(tag):
            c = draw_debug(img, res, status="%s t=%.2fs %s" % (name, rel, tag), confirmed=conf,
                           count=tracker.count, proc_ms=det_ms, orig_height=orig_size[1],
                           track_y=ty, hold=tracker.hold)
            return c

        if row is not None:
            s = state[row["idx"]]
            if tail:
                s["n_tail"] += 1
                s["n_tail_conf"] += int(conf)
            else:
                s["n"] += 1
                s["n_conf"] += int(conf)
                s["n_raw"] += int(res["found"])
                if label == "target":   # 確定位置が 0.1 以上跳んだ回数（別の境界への乗り換えの目安）
                    if conf and s["prev_y"] is not None and abs(ny - s["prev_y"]) > 0.1:
                        s["jumps"] += 1
                    s["prev_y"] = ny if conf else s["prev_y"]
            if label == "target":
                if conf and s["first"] is None:
                    canvas = debug_image("FIRST")
                    fn = "first_%s_%07.2fs.jpg" % (name, rel)
                    cv2.imwrite(os.path.join(img_dir, fn), canvas)
                    s["first"] = dict(t=rel, ny=ny, py=ny * orig_size[1], image=fn)
                if res["found"] and s["first_raw"] is None:
                    s["first_raw"] = dict(t=rel, ny=raw_ny)
            elif label == "negative":
                if conf and not s["in_event"]:
                    canvas = debug_image("FALSE POSITIVE")
                    fn = "fp_%s_%07.2fs.jpg" % (name, rel)
                    cv2.imwrite(os.path.join(img_dir, fn), canvas)
                    s["events"].append(dict(t_start=rel, t_end=rel, ny=ny, py=ny * orig_size[1],
                                            n=1, image=fn))
                elif conf and s["in_event"]:
                    s["events"][-1]["t_end"] = rel
                    s["events"][-1]["n"] += 1
                s["in_event"] = conf
        else:
            unl["n"] += 1
            unl["n_conf"] += int(conf)

        frames.append(dict(frame=k, t=rel, stamp=msg.header.stamp.to_sec(), label=label,
                           tail=int(tail), detected_raw=int(res["found"]), detected=int(conf),
                           hold=int(conf and tracker.hold), count=tracker.count,
                           normalized_y=ny,
                           pixel_y=(ny * orig_size[1]) if conf else None,
                           raw_normalized_y=raw_ny, n_cands=len(res["cands"]),
                           decode_ms=dec_ms, detect_ms=det_ms))

        if args.video:
            if canvas is None:
                canvas = debug_image(label + (" tail" if tail else ""))
            if writer is None:
                vh, vw = canvas.shape[0] // 2 * 2, canvas.shape[1] // 2 * 2   # mp4 は偶数サイズが安全
                writer = cv2.VideoWriter(os.path.join(out_dir, "debug.mp4"),
                                         cv2.VideoWriter_fourcc(*"mp4v"), args.video_fps, (vw, vh))
            if canvas.shape[:2] != (vh, vw):
                canvas = cv2.resize(canvas, (vw, vh))
            writer.write(canvas)

        if (k + 1) % 50 == 0:
            print("    %d / %d" % (k + 1, total))
    bag.close()
    if writer is not None:
        writer.release()

    with open(os.path.join(out_dir, "frames.csv"), "w", newline="") as f:
        cols = ["frame", "t", "stamp", "label", "tail", "detected_raw", "detected", "hold", "count",
                "normalized_y", "pixel_y", "raw_normalized_y", "n_cands", "decode_ms", "detect_ms"]
        w = csv.writer(f, lineterminator="\n")
        w.writerow(cols)
        for fr in frames:
            w.writerow([fmt(fr[c], "%.4f") if isinstance(fr[c], float) or fr[c] is None else fr[c]
                        for c in cols])

    results, events = [], []
    for s in state.values():
        r = s["row"]
        results.append(dict(name=name, **r, n=s["n"], n_conf=s["n_conf"], n_raw=s["n_raw"],
                            n_tail=s["n_tail"], n_tail_conf=s["n_tail_conf"], jumps=s["jumps"],
                            first=s["first"], first_raw=s["first_raw"], events=s["events"]))
        if s["first"]:
            events.append(dict(bag=name, kind="first_detection", label=r["label"],
                               t_start=s["first"]["t"], t_end=s["first"]["t"],
                               normalized_y=s["first"]["ny"], pixel_y=s["first"]["py"],
                               frames=1, image=os.path.join(name, "images", s["first"]["image"]),
                               note=r["note"]))
        for e in s["events"]:
            events.append(dict(bag=name, kind="false_positive", label=r["label"],
                               t_start=e["t_start"], t_end=e["t_end"], normalized_y=e["ny"],
                               pixel_y=e["py"], frames=e["n"],
                               image=os.path.join(name, "images", e["image"]), note=r["note"]))
    results.append(dict(name=name, bag=bag_id, idx=-1, label="unlabeled", start=None, end=None,
                        edge_type="", view="", place="", note="", n=unl["n"], n_conf=unl["n_conf"],
                        n_raw=0, n_tail=0, n_tail_conf=0, jumps=0, first=None, first_raw=None, events=[]))
    plot_timeline(os.path.join(out_dir, "timeline.png"), name, frames, rows, args.tail_exclude)
    det_ms = np.array([fr["detect_ms"] for fr in frames]) if frames else np.zeros(1)
    dec_ms = np.array([fr["decode_ms"] for fr in frames]) if frames else np.zeros(1)
    print("    処理時間: 検知 平均 %.0fms / 最大 %.0fms、デコード 平均 %.0fms"
          % (det_ms.mean(), det_ms.max(), dec_ms.mean()))
    for fr in frames:
        fr["bag_file"] = name
    return results, events, frames


# --------------------------------------------------------------------------
def plot_timeline(path, name, frames, rows, tail_exclude):
    """時間と検知位置のグラフ（背景の色: 緑=target, 赤=negative, 灰=ignore）"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("    (matplotlib が無いのでグラフは省略)")
        return
    if not frames:
        return
    fig, ax = plt.subplots(figsize=(12, 4))
    t = np.array([f["t"] for f in frames])
    raw = np.array([np.nan if f["raw_normalized_y"] is None else f["raw_normalized_y"] for f in frames])
    conf = np.array([np.nan if f["normalized_y"] is None else f["normalized_y"] for f in frames])
    y_top = 0.55 if np.all(np.isnan(raw)) else min(0.55, float(np.nanmin(raw)) - 0.05)
    colors = {"target": "#c8f0c8", "negative": "#f5c6c6", "ignore": "#dddddd"}
    for r in rows:
        ax.axvspan(r["start"], r["end"], color=colors.get(r["label"], "#ffffff"), zorder=0)
        if r["label"] == "target" and tail_exclude > 0:
            ax.axvspan(max(r["start"], r["end"] - tail_exclude), r["end"], color="#9ad29a", zorder=0)
        if r["end"] > t.min() and r["start"] < t.max():
            ax.text(max(r["start"], t.min()), y_top + 0.01, " " + r["label"], fontsize=8,
                    va="top", color="#555555")
    ax.plot(t, raw, ".", color="#999999", ms=4, label="raw (single frame)")
    ax.plot(t, conf, ".", color="#1f4fd1", ms=5, label="detected (confirmed)")
    ax.set_ylim(1.0, y_top)    # 画像と同じく上が遠い（小さい値）、下が近い（1 に近い値）
    ax.set_xlim(t.min(), t.max())
    ax.set_xlabel("time in bag [s]")
    ax.set_ylabel("normalized_y (0=top, 1=bottom)")
    ax.set_title(name)
    ax.grid(alpha=0.3)
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def pct(a, b):
    return "%5.1f%% (%d/%d)" % (100.0 * a / b, a, b) if b else "   -   (0/0)"


def main():
    ap = argparse.ArgumentParser(description="rosbag で道路端検知を評価する",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bags", nargs="+", required=True, help="評価する bag（ワイルドカード可）")
    ap.add_argument("--scenes", required=True, help="場面台帳 scenes.csv")
    ap.add_argument("--out", default="eval_out", help="出力先ディレクトリ")
    ap.add_argument("--topic", default="/usb_cam/image_raw/compressed", help="画像トピック")
    ap.add_argument("--config", default=DEFAULT_CONFIG, help="パラメータの YAML（既定: config/c2.yaml）")
    ap.add_argument("--num-detection", type=int, default=None,
                    help="連続検出のフレーム数（既定: YAML の detection_count_th）")
    ap.add_argument("--max-miss", type=int, default=None,
                    help="確定後に許す途切れのフレーム数（既定: YAML の max_miss）")
    ap.add_argument("--tail-exclude", type=float, default=1.0,
                    help="target 区間の end_s 直前で、検知率から除く秒数（既定 1.0）")
    ap.add_argument("--video", action="store_true", help="全フレームのデバッグ動画を保存する")
    ap.add_argument("--video-fps", type=float, default=10.0)
    args = ap.parse_args()

    bags = []
    for b in args.bags:
        bags.extend(sorted(glob.glob(os.path.expanduser(b))))
    if not bags:
        sys.exit("bag が見つかりません: %s" % args.bags)
    cfg, det_params = load_config(args.config)
    count_th = args.num_detection if args.num_detection is not None else int(cfg.get("detection_count_th", 3))
    max_jump = float(cfg.get("max_jump", 0.06))
    max_miss = args.max_miss if args.max_miss is not None else int(cfg.get("max_miss", 2))
    trk_extra = dict(gate=float(cfg.get("track_gate", 0.03)), support_step=float(cfg.get("support_step", 1.0)),
                     far_switch=float(cfg.get("far_switch", 0.03)))
    for k, v in (("exit_y", 0.93), ("exit_band", 0.12), ("pending_gate", 0.02), ("pending_step", 2.0)):   # v0.2.1
        trk_extra[k] = float(cfg.get(k, v))
    trk_extra["pending_max"] = int(cfg.get("pending_max", 5))
    trk_extra["last_gate"] = float(cfg.get("last_gate", 0.02))           # v0.2.2
    scenes = load_scenes(args.scenes)
    os.makedirs(args.out, exist_ok=True)

    print("設定: %s / 連続検出 %d フレーム / 途切れ許容 %d フレーム / end_s 直前 %.1f 秒を別扱い"
          % (args.config, count_th, max_miss, args.tail_exclude))
    all_res, all_ev, all_fr = [], [], []
    for path in bags:
        bid = bag_id_of(path)
        rows = [r for r in scenes if r["bag"] == bid]
        if not rows:
            print("  ! %s: scenes.csv に bag %s の行がありません（全フレーム unlabeled で評価）"
                  % (os.path.basename(path), bid))
        res, ev, fr = evaluate_bag(path, bid, rows, args, det_params, count_th, max_jump, max_miss, trk_extra)
        all_res.extend(res)
        all_ev.extend(ev)
        all_fr.extend(fr)

    # ---------------- 全 bag のフレームごとの結果（共有しやすいよう 1 ファイルに） ----------------
    with open(os.path.join(args.out, "frames_all.csv"), "w", newline="") as f:
        cols = ["bag_file", "frame", "t", "stamp", "label", "tail", "detected_raw", "detected", "hold",
                "count", "normalized_y", "pixel_y", "raw_normalized_y", "n_cands", "decode_ms", "detect_ms"]
        w = csv.writer(f, lineterminator="\n")
        w.writerow(cols)
        for fr in all_fr:
            w.writerow([fmt(fr[c], "%.4f") if isinstance(fr[c], float) or fr[c] is None else fr[c]
                        for c in cols])

    # ---------------- 区間ごとの結果 ----------------
    with open(os.path.join(args.out, "scenes_result.csv"), "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["bag_file", "bag", "label", "start_s", "end_s", "edge_type", "view", "place",
                    "frames", "detected_frames", "detection_rate", "raw_detected_frames",
                    "tail_frames", "tail_detected_frames",
                    "first_detection_t", "first_detection_normalized_y", "first_detection_pixel_y",
                    "first_detection_delay_s", "first_detection_lead_s",
                    "position_jumps", "false_positive_events", "note"])
        for r in all_res:
            first = r["first"]
            w.writerow([r["name"], r["bag"], r["label"], fmt(r["start"], "%.2f"), fmt(r["end"], "%.2f"),
                        r["edge_type"], r["view"], r["place"], r["n"], r["n_conf"],
                        fmt(r["n_conf"] / float(r["n"]) if r["n"] else None, "%.3f"), r["n_raw"],
                        r["n_tail"], r["n_tail_conf"],
                        fmt(first["t"] if first else None, "%.2f"),
                        fmt(first["ny"] if first else None, "%.3f"),
                        fmt(first["py"] if first else None, "%.0f"),
                        fmt(first["t"] - r["start"] if first else None, "%.2f"),
                        fmt(r["end"] - first["t"] if first else None, "%.2f"),
                        r["jumps"], len(r["events"]), r["note"]])

    with open(os.path.join(args.out, "events.csv"), "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["bag_file", "kind", "label", "t_start", "t_end", "frames",
                    "normalized_y", "pixel_y", "image", "note"])
        for e in sorted(all_ev, key=lambda e: (e["bag"], e["t_start"])):
            w.writerow([e["bag"], e["kind"], e["label"], "%.2f" % e["t_start"], "%.2f" % e["t_end"],
                        e["frames"], fmt(e["normalized_y"], "%.3f"), fmt(e["pixel_y"], "%.0f"),
                        e["image"], e["note"]])

    # ---------------- まとめ ----------------
    L = []
    L.append("=" * 72)
    L.append("道路端検知 評価結果  (連続検出 %d フレーム / 途切れ許容 %d フレーム / end_s 直前 %.1f 秒は別扱い)"
             % (count_th, max_miss, args.tail_exclude))
    L.append("=" * 72)
    tg = [r for r in all_res if r["label"] == "target"]
    ng = [r for r in all_res if r["label"] == "negative"]
    L.append("[対象場面の検知率]  全体 %s" % pct(sum(r["n_conf"] for r in tg), sum(r["n"] for r in tg)))
    for key, title in (("view", "向き"), ("edge_type", "道路端の種類")):
        vals = sorted(set(r[key] for r in tg))
        for v in vals:
            sub = [r for r in tg if r[key] == v]
            L.append("    %-12s %-12s %s" % (title, v or "(空欄)", pct(sum(r["n_conf"] for r in sub), sum(r["n"] for r in sub))))
    L.append("    参考: end_s 直前 %.1f 秒 %s" % (args.tail_exclude, pct(sum(r["n_tail_conf"] for r in tg), sum(r["n_tail"] for r in tg))))
    L.append("")
    L.append("[区間ごとの検知率と初回検知]")
    for r in tg:
        f0 = r["first"]
        L.append("  %-28s %6.2f-%6.2fs %-11s %-6s 検知率 %s  跳び %d 回  初回 %s" % (
            r["name"], r["start"], r["end"], r["edge_type"], r["view"],
            pct(r["n_conf"], r["n"]), r["jumps"],
            ("t=%.2fs normalized_y=%.3f (開始から %.2fs, end_s の %.2fs 前)" %
             (f0["t"], f0["ny"], f0["t"] - r["start"], r["end"] - f0["t"])) if f0 else "なし"))
    L.append("  ※「跳び」= 確定位置が 0.1 以上変わった回数。別の境界（奥の道路端など）への乗り換えの目安")
    L.append("")
    nf = sum(r["n"] for r in ng)
    L.append("[非対象区間の誤検知]  フレーム %s / 回数 %d 回" % (pct(sum(r["n_conf"] for r in ng), nf),
                                                        sum(len(r["events"]) for r in ng)))
    for r in ng:
        for e in r["events"]:
            L.append("  %-28s t=%.2f-%.2fs (%dフレーム) normalized_y=%.3f  %s  [%s]" % (
                r["name"], e["t_start"], e["t_end"], e["n"], e["ny"], e["image"], r["note"]))
    un = [r for r in all_res if r["label"] == "unlabeled"]
    if sum(r["n"] for r in un):
        L.append("")
        L.append("[台帳に無い時刻]  検知 %s" % pct(sum(r["n_conf"] for r in un), sum(r["n"] for r in un)))
    L.append("")
    L.append("出力: %s" % os.path.abspath(args.out))
    text = "\n".join(L)
    print("\n" + text)
    with open(os.path.join(args.out, "summary.txt"), "w") as f:
        f.write(text + "\n")


if __name__ == "__main__":
    main()

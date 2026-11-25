#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
road_edge_detector_node.py

C2 カメラの圧縮画像から、横断歩道へ進入する手前側の道路端（歩道と車道の境界）を検知する ROS ノード。
第1段階では、実距離の推定と停止制御は行わない（/cmd_vel や停止サービスは操作しない）。

購読
  ~image_topic で指定したトピック (sensor_msgs/CompressedImage)
    既定: /usb_cam/image_raw/compressed
  ~enable (std_msgs/Bool)   false の間は検知を止め、/road_edge/detected = false を配信する（v0.2）
                            横断歩道に近づくときだけ有効にする使い方を想定（既定は有効）
  ~node_topic で指定したトピック（v0.3。ナビゲーションの現在のノード番号。LocalizationDiagnostics.current_node_idx）
    既定: /toponav/localization_diagnostics
    use_target_gate が true のとき、targets に書いた道路端のノードの手前だけ検知結果を配信する
    （v0.3.2）targets の各項目の detector: / tracker: / gate: に書いた値は、その道路端の判定区間の中だけ使う

処理（検知・連続検出・絞り込み）は road_edge_detector.pipeline.RoadEdgePipeline にまとめてあり（v0.3.2）、
このノードは画像とノード番号を渡して、結果をトピックに配信するだけ。評価ツール（tools/run_full_bag.py など）も同じクラスを使う

配信（launch の既定どおりノード名 road_edge で起動した場合）
  /road_edge/detected                 std_msgs/Bool     連続検出で確定した道路端があるか
                                                        （確定後は max_miss フレームまでの途切れを許し、位置を保つ）
  /road_edge/pixel_y                  std_msgs/Float32  画像中央の列での道路端の縦位置 [px]（元画像の座標）。未検知は NaN
  /road_edge/normalized_y             std_msgs/Float32  pixel_y / 元画像の高さ（0〜1）。未検知は NaN
  /road_edge/detected_raw             std_msgs/Bool     そのフレーム単体で検知したか（連続検出の条件をかける前。ノード番号では絞らない）
  /road_edge/target                   std_msgs/String   いま判定している道路端の名前（targets の name。判定区間の外では空）（v0.3）
  /road_edge/target_passed            std_msgs/Bool     その道路端を通過したと判断したか（v0.3）
  /road_edge/debug_image/compressed   sensor_msgs/CompressedImage
                                      visualize が true かつ購読者がいるとき。header は入力画像と同じ
"""
import json
import os
import sys
from placenav_viz_msgs.msg import LocalizationDiagnostics
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String

try:
    from road_edge_detector.detector import DEFAULT_PARAMS, decode_compressed
    from road_edge_detector.visualize import draw_debug
    from road_edge_detector.pipeline import RoadEdgePipeline, TRACKER_KEYS
except ImportError:  # catkin build 前でも動くように、パッケージの src を直接読む
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
    from road_edge_detector.detector import DEFAULT_PARAMS, decode_compressed
    from road_edge_detector.visualize import draw_debug
    from road_edge_detector.pipeline import RoadEdgePipeline, TRACKER_KEYS

# 検知・連続検出・絞り込みに使うパラメータの名前（c2.yaml と同じ。~ 以下から読む）
GATE_PARAM_KEYS = ("use_target_gate", "targets", "target_before", "target_after", "pass_y", "pass_jump",
                   "pass_arm_before", "pass_from", "acq_y", "reacq_back", "reacq_fwd", "pub_min_y")

NAN = float("nan")


class RoadEdgeDetectorNode(object):
    def __init__(self):
        # 検知・連続検出・絞り込みのパラメータ（YAML で ~ 以下に読み込まれたもの）。処理は RoadEdgePipeline にまとめた（v0.3.2）
        cfg = {}
        names = list(DEFAULT_PARAMS) + [k for k, _, _ in TRACKER_KEYS] + list(GATE_PARAM_KEYS)
        for k in names:
            v = rospy.get_param("~" + k, None)
            if v is not None:
                cfg[k] = v
        target = rospy.get_param("~override_target_node", None)
        if target is not None:
            if target < 0:
                raise ValueError("target_node must be nonnegative")
            cfg["targets"] = [dict(name="node_%d" % target, node=target)]
        for side in ("before", "after"):
            width = rospy.get_param("~override_target_" + side, None)
            if width is not None:
                if width < 0:
                    raise ValueError("target_" + side + " must be nonnegative")
                cfg["target_" + side] = width
                for entry in cfg["targets"]:
                    entry[side] = width
        self.pipe = RoadEdgePipeline(cfg)
        self.detector = self.pipe.base_detector
        self.visualize = bool(rospy.get_param("~visualize", True))
        self.decode_reduction = rospy.get_param("~decode_reduction", 1)
        self.enabled = bool(rospy.get_param("~start_enabled", True))
        self.debug_quality = int(rospy.get_param("~debug_jpeg_quality", 80))
        self.debug_width = int(rospy.get_param("~debug_width", 720))
        image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw/compressed")

        # 判定したい道路端だけを出す（v0.3。ナビの現在のノード番号で選ぶ）
        self.gate = self.pipe.gate
        self.node_idx = None
        self.node_recv = None
        if self.gate is not None:
            self.node_topic = rospy.get_param("~node_topic", "/toponav/localization_diagnostics")
            self.node_timeout = float(rospy.get_param("~node_timeout", 3.0))
            self.sub_node = rospy.Subscriber(self.node_topic, LocalizationDiagnostics, self.on_node, queue_size=1)
        self.prev_target = ""

        self.orig_size = None           # 元画像の (width, height)。最初の画像で決まる
        self.prev_confirmed = False
        self.proc_ms = []

        self.pub_detected = rospy.Publisher("~detected", Bool, queue_size=1)
        self.pub_detected_raw = rospy.Publisher("~detected_raw", Bool, queue_size=1)
        self.pub_pixel_y = rospy.Publisher("~pixel_y", Float32, queue_size=1)
        self.pub_norm_y = rospy.Publisher("~normalized_y", Float32, queue_size=1)
        self.pub_debug = rospy.Publisher("~debug_image/compressed", CompressedImage, queue_size=1)
        self.pub_status = rospy.Publisher("~gate_status", String, queue_size=1)
        self.pub_target = rospy.Publisher("~target", String, queue_size=1)
        self.pub_target_passed = rospy.Publisher("~target_passed", Bool, queue_size=1)
        # 大きな画像を遅れなく受け取るため buff_size を大きくし、最新の 1 枚だけ処理する
        self.sub = rospy.Subscriber(image_topic, CompressedImage, self.callback,
                                    queue_size=1, buff_size=2 ** 24)
        self.sub_enable = rospy.Subscriber("~enable", Bool, self.on_enable, queue_size=1)

        p = self.detector.p
        rospy.loginfo("[road_edge] 購読: %s", image_topic)
        rospy.loginfo("[road_edge] ROI(%s) 上左%s 上右%s 下左%s 下右%s / 処理幅 %dpx / 連続検出 %d フレーム"
                      " / 途切れ許容 %d フレーム / 横断歩道の白線: %s / 検知: %s",
                      p["roi_units"], p["roi_upper_left"], p["roi_upper_right"],
                      p["roi_lower_left"], p["roi_lower_right"], p["process_width"],
                      self.pipe.tracker.count_th, self.pipe.tracker.max_miss,
                      "使う" if p.get("cw_enable", True) else "使わない",
                      "有効" if self.enabled else "停止中（~enable に true が来るまで）")
        if self.gate is not None:
            rospy.loginfo("[road_edge] 判定する道路端: %s（ノード番号 %s、判定区間 -%d〜+%d）",
                          ", ".join(t["name"] for t in self.gate.targets),
                          ", ".join(str(t["node"]) for t in self.gate.targets),
                          int(rospy.get_param("~target_before", 3)), int(rospy.get_param("~target_after", 4)))
            special = ["%s（%s）" % (t.get("name", "?"), " / ".join(k for k in ("detector", "tracker", "gate") if t.get(k)))
                       for t in self.pipe.cfg.get("targets", []) if any(t.get(k) for k in ("detector", "tracker", "gate"))]
            if special:
                rospy.loginfo("[road_edge] 道路端ごとの設定: %s", ", ".join(special))
        else:
            rospy.loginfo("[road_edge] ノード番号では絞りません（use_target_gate: false）")

    # ------------------------------------------------------------------
    def on_node(self, msg):
        self.node_idx = msg.current_node_idx
        self.node_recv = time.time()

    def on_enable(self, msg):
        if bool(msg.data) != self.enabled:
            self.enabled = bool(msg.data)
            self.pipe.reset()
            self.prev_confirmed = False
            rospy.loginfo("[road_edge] 検知を%sしました" % ("開始" if self.enabled else "停止"))

    def callback(self, msg):
        if not self.enabled:            # 停止中は画像を処理せず「検知なし」だけ配信する
            self.pub_status.publish(String(data=json.dumps(dict(
                current_node=None, gate_enabled=self.gate is not None, target=None,
                in_window=False, raw=False, detected=False))))
            self.pub_detected.publish(Bool(data=False))
            self.pub_detected_raw.publish(Bool(data=False))
            self.pub_pixel_y.publish(Float32(data=NAN))
            self.pub_norm_y.publish(Float32(data=NAN))
            return
        t0 = time.time()
        img, orig_size = decode_compressed(msg.data, self.detector.p["process_width"],
                                           self.decode_reduction, self.orig_size)
        if img is None:
            rospy.logwarn_throttle(5.0, "[road_edge] 画像をデコードできません (format=%s)" % msg.format)
            return
        if self.orig_size is None:
            rospy.loginfo("[road_edge] 入力画像 %dx%d を %dx%d でデコードし、幅 %dpx に縮小して処理します",
                          orig_size[0], orig_size[1], img.shape[1], img.shape[0],
                          self.detector.p["process_width"])
        self.orig_size = orig_size

        gate = self.gate
        node = None
        if gate is not None:
            fresh = self.node_recv is not None and (time.time() - self.node_recv) <= self.node_timeout
            node = self.node_idx if fresh else None
        self.pipe.begin(node)                     # 判定区間に入ったら、その道路端の設定で連続検出をやり直す
        res = self.pipe.detector().detect(img, orig_size)   # 道路端ごとの detector: の設定があればそれを使う
        o = self.pipe.update(res, stamp=msg.header.stamp.to_sec(), orig_height=orig_size[1])
        confirmed = o.detected
        out_y = o.normalized_y
        if confirmed:
            ny = float(out_y)            # 連続検出で確定している位置（途切れ中は直前の位置）
            py = ny * orig_size[1]
        else:
            ny = py = NAN

        current = gate.cur if gate is not None and node is not None else None
        status = dict(current_node=node, gate_enabled=gate is not None,
                      target=current["node"] if current else None,
                      in_window=bool(current), raw=bool(res["found"]), detected=bool(confirmed))
        self.pub_status.publish(String(data=json.dumps(status)))
        self.pub_detected.publish(Bool(data=confirmed))
        self.pub_detected_raw.publish(Bool(data=bool(res["found"])))
        self.pub_pixel_y.publish(Float32(data=py))
        self.pub_norm_y.publish(Float32(data=ny))
        if gate is not None:
            self.pub_target.publish(String(data=gate.name))
            self.pub_target_passed.publish(Bool(data=gate.state == "passed"))
            if gate.name != self.prev_target:
                if gate.name:
                    rospy.loginfo("[road_edge] 道路端 %s の判定区間に入りました（ノード %s）", gate.name, gate.node)
                else:
                    rospy.loginfo("[road_edge] 道路端の判定区間を出ました（ノード %s）", gate.node)
                self.prev_target = gate.name

        stamp = msg.header.stamp.to_sec()
        if confirmed and not self.prev_confirmed:
            rospy.loginfo("[road_edge] 道路端を検知: normalized_y=%.3f pixel_y=%.0f (stamp %.3f)", ny, py, stamp)
        elif self.prev_confirmed and not confirmed:
            rospy.loginfo("[road_edge] 道路端の検知が途切れました (stamp %.3f)", stamp)
        self.prev_confirmed = confirmed

        proc_ms = (time.time() - t0) * 1000.0
        if self.visualize and self.pub_debug.get_num_connections() > 0:
            status = ""
            if gate is not None:
                status = "node %s %s %s" % ("-" if gate.node is None else gate.node, gate.name or "-",
                                            gate.state if gate.name else "")
                if o.near_ignored:
                    status += " (near line ignored)"
            tr = self.pipe.tracker
            canvas = draw_debug(img, res, status=status, confirmed=confirmed, count=tr.count,
                                proc_ms=proc_ms, orig_height=orig_size[1], max_width=self.debug_width,
                                track_y=out_y, hold=tr.hold, supported=tr.supported, handover=tr.handover,
                                track_pts=o.track_pts)
            ok, enc = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, self.debug_quality])
            if ok:
                out = CompressedImage()
                out.header = msg.header          # 入力画像と同じ stamp と frame_id
                out.format = "jpeg"
                out.data = enc.tobytes()
                self.pub_debug.publish(out)

        self.proc_ms.append(proc_ms)
        if len(self.proc_ms) >= 100:
            a = np.array(self.proc_ms)
            rospy.loginfo("[road_edge] 処理時間 平均 %.0fms / 最大 %.0fms（直近 %d フレーム）",
                          a.mean(), a.max(), len(a))
            self.proc_ms = []


def main():
    rospy.init_node("road_edge")
    RoadEdgeDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()

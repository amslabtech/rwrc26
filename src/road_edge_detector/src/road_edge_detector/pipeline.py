# -*- coding: utf-8 -*-
"""
road_edge_detector.pipeline（v0.3.2）

1 枚の画像の検知（RoadEdgeDetector）・連続検出（EdgeTracker）・判定したい道路端だけを出す絞り込み（TargetGate）を
まとめて行うクラス。ROS に依存しないので、ROS ノード・評価ツール・ほかのプログラムから同じ処理を呼べる。

    import yaml, cv2
    from road_edge_detector.pipeline import RoadEdgePipeline
    pipe = RoadEdgePipeline(yaml.safe_load(open("config/c2.yaml")))
    out = pipe.process(img, stamp=t, node=current_node_idx)   # img: BGR 画像（numpy）
    if out.detected:
        print(out.target, out.normalized_y)                   # 判定している道路端と、画像中央の列での位置

道路端ごとの設定: config の targets の各項目に detector: / tracker: / gate: を書くと、その道路端の判定区間の中だけ、
検知器・連続検出・絞り込みのパラメータを変える（例: E06 の斜めの道路端）。名前は c2.yaml と同じ。
"""
import copy

from .detector import DEFAULT_PARAMS, EdgeTracker, RoadEdgeDetector
from .target_gate import TargetGate
from .visualize import published_pts

# c2.yaml の名前 → EdgeTracker の引数名
TRACKER_KEYS = (("detection_count_th", "count_th", int), ("max_jump", "max_jump", float), ("max_miss", "max_miss", int),
                ("track_gate", "gate", float), ("support_step", "support_step", float),
                ("far_switch", "far_switch", float), ("exit_y", "exit_y", float), ("exit_band", "exit_band", float),
                ("pending_gate", "pending_gate", float), ("pending_step", "pending_step", float),
                ("pending_max", "pending_max", int), ("last_gate", "last_gate", float),
                ("support_up", "support_up", float))


def detector_params(cfg, overrides=None):
    p = {k: cfg[k] for k in DEFAULT_PARAMS if k in cfg}
    for k, v in (overrides or {}).items():
        if k not in DEFAULT_PARAMS:
            raise ValueError("detector: に知らない名前 %s があります" % k)
        p[k] = v
    return p


def tracker_kwargs(cfg, overrides=None):
    c = dict(cfg)
    names = set(k for k, _, _ in TRACKER_KEYS)
    for k, v in (overrides or {}).items():
        if k not in names:
            raise ValueError("tracker: に知らない名前 %s があります（使えるのは %s）" % (k, ", ".join(sorted(names))))
        c[k] = v
    return {arg: conv(c[k]) for k, arg, conv in TRACKER_KEYS if k in c}


def gate_kwargs(cfg):
    kw = dict(before=int(cfg.get("target_before", 3)), after=int(cfg.get("target_after", 4)),
              pass_y=float(cfg.get("pass_y", 0.90)), pass_jump=float(cfg.get("pass_jump", 0.15)),
              arm_before=int(cfg.get("pass_arm_before", 1)), pass_from=float(cfg.get("pass_from", 0.80)),
              acq_y=float(cfg.get("acq_y", 0.75)), reacq_back=float(cfg.get("reacq_back", 0.03)),
              reacq_fwd=float(cfg.get("reacq_fwd", 0.08)), pub_min_y=float(cfg.get("pub_min_y", 0.655)))
    return kw


class PipelineOutput(object):
    """1 フレームの結果"""
    __slots__ = ("detected", "normalized_y", "pixel_y", "detected_raw", "raw_y", "target", "state", "passed",
                 "near_ignored", "confirmed_ungated", "y_ungated", "track_pts", "res", "node")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))


class RoadEdgePipeline(object):
    def __init__(self, cfg, use_target_gate=None):
        """
        cfg: c2.yaml を読み込んだ dict
        use_target_gate: None なら cfg の use_target_gate（既定 true）。false ならノード番号で絞らない（v0.2 と同じ）
        """
        self.cfg = copy.deepcopy(cfg or {})
        if use_target_gate is None:
            use_target_gate = bool(self.cfg.get("use_target_gate", True))
        self.base_detector = RoadEdgeDetector(detector_params(self.cfg))
        self._detectors = {"": self.base_detector}
        self.gate = TargetGate(self.cfg.get("targets", []), **gate_kwargs(self.cfg)) if use_target_gate else None
        if self.gate is not None:       # 道路端ごとの検知器を先に作っておく（設定の誤りを起動時に見つけるため）
            for t in self.gate.targets:
                if t["detector"]:
                    self._detectors[t["name"]] = RoadEdgeDetector(detector_params(self.cfg, t["detector"]))
                tracker_kwargs(self.cfg, t["tracker"])
        self.tracker = EdgeTracker(**tracker_kwargs(self.cfg))
        self._trk_for = ""
        self._pts = None
        self._pts_y = None

    # ------------------------------------------------------------------
    @property
    def target(self):
        return self.gate.name if self.gate is not None else ""

    def detector(self):
        """いまの判定区間の道路端に使う検知器（begin の後に呼ぶ）"""
        return self._detectors.get(self.target, self.base_detector)

    def begin(self, node=None):
        """
        画像 1 枚ごとに、検知の前に呼ぶ。node: ナビの現在のノード番号（届いていなければ None）
        新しい判定区間に入ったら、その道路端の設定で連続検出をやり直す
        """
        if self.gate is None:
            return ""
        self.gate.set_node(node)
        new = self.gate.begin_frame()
        name = self.gate.name
        if new or name != self._trk_for:
            if new or self.gate.tracker_overrides or self._trk_for:
                self.tracker = EdgeTracker(**tracker_kwargs(self.cfg, self.gate.tracker_overrides))
            self._trk_for = name
        return name

    def update(self, res, stamp=None, orig_height=None):
        """検知の結果 res（RoadEdgeDetector.detect の戻り値）で、連続検出と絞り込みを進める"""
        conf, ty = self.tracker.update(res["found"], res["normalized_y"], res["support"], res["trusted"], stamp=stamp)
        near_ignored = False
        if self.gate is not None:
            out, oy = self.gate.end_frame(conf, ty, self.tracker.track_id, self.tracker.y0)
            near_ignored = bool(self.gate.blocked)
            state = self.gate.state
        else:
            out, oy = conf, (ty if conf else None)
            state = "approach" if conf else "off"
        out = bool(out)
        if out:                          # 配信している線全体（デバッグ画像のマゼンタ）
            self._pts = published_pts(res, oy, self._pts, self._pts_y)
            self._pts_y = oy
        else:
            self._pts = self._pts_y = None
        H = orig_height or (res.get("image_size") or (None, None))[1]
        return PipelineOutput(detected=out, normalized_y=float(oy) if out else None,
                              pixel_y=(float(oy) * H if (out and H) else None),
                              detected_raw=bool(res["found"]), raw_y=res["normalized_y"], target=self.target,
                              state=state, passed=(state == "passed"), near_ignored=near_ignored,
                              confirmed_ungated=bool(conf), y_ungated=(ty if conf else None), track_pts=self._pts,
                              res=res, node=(self.gate.node if self.gate is not None else None))

    def process(self, img, orig_size=None, stamp=None, node=None):
        """begin → 検知 → update をまとめて行う。img: BGR 画像"""
        self.begin(node)
        res = self.detector().detect(img, orig_size)
        H = orig_size[1] if orig_size else img.shape[0]
        return self.update(res, stamp=stamp, orig_height=H)

    def reset(self):
        """連続検出をやり直す（~enable で止めたときなど）"""
        self.tracker.reset()
        self._pts = self._pts_y = None

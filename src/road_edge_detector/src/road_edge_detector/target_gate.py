# -*- coding: utf-8 -*-
"""
判定したい道路端だけを出すためのゲート（v0.3 / v0.3.1 / v0.3.2）

ナビゲーションの「現在のノード番号」（地図のノード）を使い、判定したい道路端のノードの手前だけ
連続検出の結果を通す。時間やフレーム番号は使わない。

  - 判定区間: 現在のノードが [node - before, node + after] にある間だけ、その道路端の検知を出す
  - 判定区間に入ったら連続検出をやり直す（判定区間の外で追っていた線を持ち込まない）
  - 通過: 確定していた道路端が pass_y より下（手前）まで来た後に見えなくなる、または
          pass_jump 以上向こうの線に移ったら「通過した」とみなし、その判定区間ではそれ以降出さない
          （道路を渡り始めた後に見える、向こう側の道路端に釣られないため）。
          ノードが node - arm_before より手前のうちは通過にしない（近づく前に歩道の中の帯が
          画像の下に出ていっても、通過にしない）
  - 一度通過した、または判定区間を出た道路端は、ノードが判定区間より rearm_back 以上手前に戻るまで再び出さない
    （ナビのノードが前後に揺れても、渡った先で同じ道路端をもう一度出さないため）
  - 遠くから追ってきた線だけ出す（v0.3.1）: 連続検出が追い始めた位置（y0）が acq_y より手前（画像の下）の線は出さない。
    判定したい道路端は、近づくにつれて画像の上から下りてくるので、いきなり手前に現れる線は歩道の中の線。
    ただし、判定区間の中で最後に出した位置の近く（-reacq_back〜+reacq_fwd）で見つけ直した線は出す
    （人に隠れて一度見失った道路端を、手前で見つけ直したとき）
  - pub_min_y より向こう（画像の上の端）の位置は出さない（v0.3.1。ROI の上の縁にかかった線は位置があいまいなため）
  - 道路端ごとの設定（v0.3.2）: targets の各項目に gate: {pass_y: ..., pub_min_y: ...} のように書くと、
    その道路端の判定区間の中だけ、上の値（pass_y, pass_jump, pass_from, arm_before, acq_y, reacq_back, reacq_fwd, pub_min_y）を変える。
    detector: / tracker: に書いた値は RoadEdgePipeline（pipeline.py）が、その判定区間の中の検知器・連続検出に使う
"""

GATE_KEYS = ("pass_y", "pass_jump", "pass_from", "arm_before", "acq_y", "reacq_back", "reacq_fwd", "pub_min_y")


class TargetGate(object):
    def __init__(self, targets, before=3, after=4, pass_y=0.90, pass_jump=0.15, arm_before=1, rearm_back=8,
                 pass_from=0.80, acq_y=0.75, reacq_back=0.03, reacq_fwd=0.08, pub_min_y=0.655):
        """
        targets: [{"name": "E01", "node": 159, ("before": 3, "after": 3, "gate": {...}, "detector": {...},
                   "tracker": {...})}, ...]
        """
        defaults = dict(pass_y=float(pass_y), pass_jump=float(pass_jump), pass_from=float(pass_from),
                        arm_before=int(arm_before), acq_y=float(acq_y), reacq_back=float(reacq_back),
                        reacq_fwd=float(reacq_fwd), pub_min_y=float(pub_min_y))
        self.targets = []
        for k, t in enumerate(targets):
            gp = dict(defaults)
            for key, v in (t.get("gate") or {}).items():
                key = "arm_before" if key == "pass_arm_before" else key      # c2.yaml と同じ名前でも書ける
                if key not in GATE_KEYS:
                    raise ValueError("targets の %s の gate: に知らない名前 %s があります（使えるのは %s）" % (
                        t.get("name", "?"), key, ", ".join(GATE_KEYS)))
                gp[key] = int(v) if key == "arm_before" else float(v)
            self.targets.append(dict(name=str(t.get("name", "T%02d" % (k + 1))), node=int(t["node"]),
                                     before=int(t.get("before", before)), after=int(t.get("after", after)),
                                     index=k + 1, gp=gp, detector=dict(t.get("detector") or {}),
                                     tracker=dict(t.get("tracker") or {})))
        self.pass_y = float(pass_y)
        self.pass_jump = float(pass_jump)
        self.pass_from = float(pass_from)
        self.arm_before = int(arm_before)
        self.rearm_back = int(rearm_back)
        self.acq_y = float(acq_y)
        self.reacq_back = float(reacq_back)
        self.reacq_fwd = float(reacq_fwd)
        self.pub_min_y = float(pub_min_y)
        self.node = None
        self.done = set()           # 通過した、または判定区間を出た道路端（index）
        self.cur = None             # いま判定区間に入っている道路端
        self.passed = False
        self.last_y = None          # 判定区間の中で最後に出した位置
        self.min_y = None           # 判定区間の中で出した位置のうち、いちばん向こう（小さい値）
        self.last_pub_y = None      # 判定区間の中で最後に出した位置（見失っても消さない。見つけ直しの判定に使う）
        self._tid = None            # いま追っている線の番号（EdgeTracker.track_id）
        self._pub = True            # その線を出してよいか（遠くから追ってきた線か）
        self.blocked = False        # このフレームで、確定した線を「手前に急に現れた線」として出さなかったか

    # ------------------------------------------------------------------
    def set_node(self, node):
        self.node = None if node is None else int(node)
        if self.node is None:
            return
        for t in self.targets:      # 判定区間より十分手前に戻ったら、もう一度出せるようにする
            if t["index"] in self.done and self.node < t["node"] - t["before"] - self.rearm_back:
                self.done.discard(t["index"])

    def _in(self, t):
        return t["node"] - t["before"] <= self.node <= t["node"] + t["after"]

    def _find(self):
        if self.cur is not None and self._in(self.cur):     # いまの判定区間の中にいる間はそのまま（通過後も）
            return self.cur
        for t in self.targets:
            if t["index"] not in self.done and self._in(t):
                return t
        return None

    def begin_frame(self):
        """
        画像 1 枚ごとに、連続検出の更新の前に呼ぶ。
        戻り値: 連続検出をやり直すべきか（新しい判定区間に入った）
        """
        if self.node is None:          # ノード番号が来ていない間は何もしない（end_frame は出さない）
            return False
        t = self._find()
        if self.cur is not None and (t is None or t["index"] != self.cur["index"]):
            self.done.add(self.cur["index"])        # 判定区間を出た
        new = t is not None and (self.cur is None or t["index"] != self.cur["index"])
        self.cur = t
        if new:
            self.passed = False
            self.last_y = None
            self.min_y = None
            self.last_pub_y = None
            self._tid = None
            self._pub = True
        return new

    def end_frame(self, confirmed, y, track_id=None, y0=None):
        """
        連続検出の更新の後に呼ぶ。戻り値: (出すか, 位置)
        track_id, y0: EdgeTracker の track_id と y0（追っている線の番号と、最初に見つけた位置）。
                      省略すると「遠くから追ってきた線か」の判定をしない（v0.3 と同じ）
        通過の判定は、確定していた線を見失った（または大きく向こうへ跳んだ）そのフレームで行う
        """
        self.blocked = False
        if self.node is None or self.cur is None or self.passed:
            return False, None
        t = self.cur
        g = t["gp"]                      # この道路端の設定（v0.3.2。書いていなければ全体の既定値）
        if confirmed and y is not None and track_id is not None and track_id != self._tid:
            self._tid = track_id         # 新しく確定した線: 出してよいかを決める
            self._pub = (y0 is None or y0 <= g["acq_y"] or
                         (self.last_pub_y is not None and
                          self.last_pub_y - g["reacq_back"] <= y0 <= self.last_pub_y + g["reacq_fwd"]))
        if confirmed and y is not None and not self._pub:
            self.blocked = True
            confirmed = False            # 手前に急に現れた線は、見えていないのと同じに扱う
        armed = (self.node >= t["node"] - g["arm_before"] and self.min_y is not None
                 and self.min_y <= g["pass_from"])       # 向こうから手前まで追ってきた線だけ
        near = self.last_y is not None and self.last_y >= g["pass_y"]
        if confirmed and y is not None:
            if near and y < self.last_y - g["pass_jump"] and armed:
                self._pass()
                return False, None
            if y < g["pub_min_y"]:       # ROI の上の端にかかった線は、まだ出さない
                return False, None
            self.last_y = float(y)
            self.last_pub_y = float(y)
            self.min_y = float(y) if self.min_y is None else min(self.min_y, float(y))
            return True, float(y)
        if near and armed:
            self._pass()
        self.last_y = None             # 見失った（通過でなければ、次の確定を待つ）
        return False, None

    def _pass(self):
        self.passed = True
        self.last_y = None
        self.done.add(self.cur["index"])

    # ------------------------------------------------------------------
    @property
    def name(self):
        return self.cur["name"] if self.cur is not None else ""

    @property
    def index(self):
        return self.cur["index"] if self.cur is not None else 0

    @property
    def state(self):
        if self.cur is None:
            return "off"
        return "passed" if self.passed else "approach"

    @property
    def detector_overrides(self):
        """いまの判定区間の道路端の detector: の設定（判定区間の外・設定なしなら空の dict）"""
        return self.cur["detector"] if self.cur is not None else {}

    @property
    def tracker_overrides(self):
        """いまの判定区間の道路端の tracker: の設定（判定区間の外・設定なしなら空の dict）"""
        return self.cur["tracker"] if self.cur is not None else {}

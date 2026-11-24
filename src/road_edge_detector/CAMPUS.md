# 学内の道路端停止試験
WHILLドライバ、MotionDecision、C2、ナビが使う深度サーバーを準備し、学内版を含むワークスペースをsourceする。別の navigation.launch を重ねて起動しない。

    roslaunch road_edge_detector campus_whill.launch topomap_dir:=whill_odometry_dkan_3.5_0.2618 target_node:=16 target_before:=2 target_after:=3

例では推定現在ノード14〜19（両端を含む）の間だけ道路端の画像判定が停止に使われる。番号と幅の単位はノード。対象番号を省略すると配布元のつくば用E01〜E14を使用。幅を省略するとYAMLの値（全体の既定は手前3・後ろ4、地点別の指定を優先）を使用。幅0も指定可能。地図を変えても番号は自動変換されない。use_target_gate:=false で区間を限定しない試験に切り替えられる。

    rostopic echo /road_edge/gate_status

current_node は推定現在ノード、in_window は有効な対象区間、raw は画像単体の検知候補、detected は区間と連続画像などを含めた最終判定。target は有効な対象の番号。現在ノード未受信・期限切れは current_node:null。位置推定の欠落中は道路端による停止を判定できない。gate_status は画像処理時に配信され、record:=true（既定）でbagにも入る。

確定するとTEBの出力を遮断しゼロを保持。検知が消えた後 rosservice call /road_edge_stop/reset で手動解除。停止後に機体前端から道路端までの距離を実測する。単独bag再生用は campus_stop.launch（既定の速度出力は /campus_stop/output）。診断トピックと圧縮画像を再生する。依存は配布元README参照。

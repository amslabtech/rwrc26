# 局所2D推定と相対ゴール姿勢（到達時XY補正・診断用）

`local_pose:=true`で、1個の平面姿勢 `(x, y, yaw)` と3×3共分散を保持し、
選択した画像ノードへの相対姿勢を`/toponav/local_goal_pose`
（`placenav_viz_msgs/LocalGoalPose`）へ出力する。既定は無効。
全ノードにガウス分布を持つ実装ではない。

VPR・1D belief・サブゴール更新・速度指令へこの推定を戻す処理はない。
`subgoal_update:=arrival`の通常到達時に、到達したノードの記録XYへ現在位置を合わせる。
向きはリセットせず、odomによる追跡を継続する。追加の起動引数はない。
`arrival` / `passage`のリカバリー確定時には、局所位置とbeliefが大きく矛盾する場合だけ
XY基準を補正する。条件と制限は後述。
OmniVLA向け制御接続、不連続後の自動再局在はまだ実装していない。

## 起動

コンテナ内で、メッセージと起動スクリプトをビルドする。

```bash
cd /home/user/ws
source /opt/ros/noetic/setup.bash
colcon build \
  --base-paths /home/user/ws/src/PlaceNav/src/placenav_viz_msgs/src/placenav_viz_msgs \
               /home/user/ws/src/PlaceNav/src/rwrc26_bringup \
  --packages-select placenav_viz_msgs rwrc26_bringup
source /home/user/ws/install/setup.bash

roslaunch rwrc26_bringup navigation_debug.launch \
  topomap_dir:=whill_odometry_tsukuba_3.5 \
  motion_model:=odometry subgoal_update:=arrival local_pose:=true \
  bag:=/home/user/bagfiles/tsukuba/20261107_111825_777874.bag \
  bag_pause:=true
```

直接起動は`navigate_whill.sh --local-pose`。odometryモードでのみ使用できる。
メッセージをビルド後、実行中のノードは再起動する。
別ターミナルもワークスペースをsourceしてから確認する。

```bash
source /home/user/ws/install/setup.bash
rostopic echo /toponav/local_goal_pose
```

### Cannot load message classと表示される場合

Dockerイメージ内には古い`placenav_viz_msgs`もある。ビルド前から開いていたターミナルや、
`/opt/ros/noetic/setup.bash`だけを読んだ環境では、新しいPythonメッセージを参照できない場合がある。
そのターミナルで、ワークスペースの設定を最後に読み直す。

```bash
source /opt/ros/noetic/setup.bash
source /home/user/ws/install/setup.bash
python3 -c 'from placenav_viz_msgs.msg import LocalGoalPose; print(LocalGoalPose._type)'
rostopic echo /toponav/local_goal_pose
```

`placenav_viz_msgs/LocalGoalPose`と表示されれば型は読み込めている。
それでもデータが来ない場合は`local_pose:=true`とbagの再生状態を確認する。
bagを一時停止している間は新しい診断も出ない。Dockerの作り直しは不要。

### RVizで見る

`local_pose:=true`で起動すると、既存launch内で`local_goal_visualizer`も起動する。
RVizの`Image Goal`表示（MarkerArray、`/toponav/local_goal_markers`）で次を表示する。

- 紫の矢印：現在の車体位置からサブゴールへの方向・位置。
- オレンジの矢印：サブゴール位置での目標画像の撮影方向。
- 白い文字：目標ID、最後のXY基準ノード、相対XY距離。

仮想地図の`robot_pose`をそのまま実行時のodom座標へ描かず、画像時刻の
`goal_pose`（車体基準）を実行時TFで`global_frame`（既定odom）へ変換する。
位置推定や速度指令へのフィードバックはない。無効・古い姿勢やTF不足時には表示を消す。
bagを一時停止すると最後の有効なマーカーを保持する。

既に起動中のナビゲーションに表示だけ追加する場合、ビルド・source後に別ターミナルで実行できる。

```bash
rosrun rwrc26_bringup local_goal_visualizer.py
```

開いているRVizではAdd → By topic → `/toponav/local_goal_markers` → MarkerArrayで追加する。
またはFile → Open Configで`rwrc26_bringup/rviz/navigation.rviz`を読み直す。
既定以外の固定フレームを使う場合は、手動起動でも`_target_frame:=フレーム名`を指定する。

## 何を仮定しているか

最初の画像で選ばれた現在ノードの担当セルに条件付け、記録された経路XYとセルの推定方向から
初期平均・共分散を作る。ノード間の状態でも、撮影位置に強制的に一致させない。
遠方の別候補との平均を取ることもせず、その候補の確率を`anchor_node_probability`へ残す。
初回ゴール画像は従来どおりすぐ出し、信頼度が高くなるまで待たない。

これは画像から2D姿勢を測定した結果ではない。初期位置合わせには仮定があり、
誤った初期ノードを局所推定だけで検出することはできない。
VPRの全体beliefは独立に維持され、局所Gaussianへ置き換えない。

以後は画像時刻に補間したodomの相対変化をSE(2)で合成する。
横へ避けた変位もXYとして残すため、迂回距離をすべて前進と扱わない。
通常の中間arrivalで画像11→12へ進むとき、到達した11を新しいXY基準とする。
その画像時刻のodomまで移動を反映してからXYを合わせ、以後はそのodomからの差分で追跡する。
新しい目標12の位置に現在位置を合わせる処理ではない。
`ADVANCED_ARRIVAL`と更新した局所姿勢の時刻が一致する場合に一度だけ適用する。
初期画像選択、`nearest`、通常の`passage`切り替え、最終停止では補正しない。
無効化された追跡を到達判定で復活させることもない。

これは「到達した時点で、そのノードの撮影位置付近にいる」と仮定し直す実験的な補正。
到達判定は独立した精密な位置観測ではなく、VPRとodomから作った1D beliefに基づく。
画像切り替えのずれや、回避中の横ずれも補正に取り込まれ、到達時には横ずれを消す。
誤差がゼロになる保証はない。向きの初期誤差・ドリフトも、このXY補正では解消しない。

### リカバリー時の位置基準

画像ゴールが置き去りになった場合と、初期のノード誤認識が訂正された場合を扱う。
`RECOVERED_FORWARD` / `RECOVERED_BACKWARD`確定時にだけ、次を確認する。

- 訂正後の現在ノードの経路進捗±`goal_eval_neighborhood_radius`に、
  `goal_recovery_threshold`以上の全体確率がある（既定±2m、80%）。
  平均進捗がこの区間内にあるセルだけを選び、セル内の区間確率も考慮する。
- その領域のbeliefから求めた経路上の平均XYと、`local_pose`のXYとの差が
  `goal_eval_neighborhood_radius`を超える（既定2m）。

両方を満たすと、その平均XYを新しい現在位置の基準とし、同じ画像時刻のodomから追跡を続ける。
ノードの撮影位置そのものや、新しいゴールの位置へ強制的に合わせる処理ではない。
平均は選ばれたメートル近傍に限定し、遠方の別候補の広い分布の裾が重なっても、その平均位置は使わない。
共分散には選択した領域のXY分布、セル内進捗分散、位置の不確実性の下限を残す。
セル内の分布は既存の平均・分散による近似で扱う。
向きとその分散は維持し、新しいXY基準に対するXY・向きの相関はクリアする。
これは同じbeliefを使った基準の置き直しであり、独立した位置観測として融合するものではない。

補正時の`LocalGoalPose.status`は`RECOVERY_POSITION_RESET`。
`anchor_node_idx`は訂正後の現在ノード、`anchor_timestamp`は補正した画像時刻になる。
次のodom更新以降は`TRACKING_POSITION_ANCHOR`。画像ゴール選択と全体beliefは変更しない。
リカバリー待ちや単独のVPR候補変更では補正せず、無効化された追跡も復活させない。

差が既定2m以内なら位置基準を維持するため、小さな回避の横ずれはそのまま残る。
一方、1D beliefだけでは大きな横迂回と基準の誤りを完全に区別できない。
2mを超える迂回で条件がそろうと横ずれを消す可能性がある。
初期の方向誤差も自動修正しないため、ゴールの方向まで正しい保証はない。

同じodom時刻に異なる姿勢が届いた場合、既存のodom履歴も連続とは扱わないよう修正した。
同一姿勢の重複は従来どおり許容する。
位置ジャンプの判定には`最大速度×時間差 + 0.05m`の許容幅を使い、
短い時刻間隔の揺れで小さな移動を不連続と誤認しにくくする。入力odomの位置・時刻は変更しない。

## 座標と共分散

`robot_pose`は記録地図の仮想座標系`map_frame=placenav_topomap`における車体姿勢。
TFは配信しない。実行時の`odom`原点・向きと一致している必要はない。
`goal_pose`は同じ画像IDの記録姿勢を現在車体から見た相対姿勢で、
`header.frame_id`は地図と一致する車体フレーム（通常`base_link`）。
位置は前方x・左y（m）、向きは反時計回りyaw（rad）。

平均の更新は `T_map_body(t) = T_map_body(t-1) * T_body(t-1)_body(t)`。
共分散は一次のヤコビアンで伝播し、移動量・旋回量に応じた過程ノイズを加える。
直線移動では旋回ノイズが途中で生じることによる位置・向きの相関も積分する。
同じ直線移動を分割しても平均・共分散が一致することをテストしている。
曲線での一次近似や画像間に失われた往復運動まで正確に再現する保証はない。

現在の仮定値は`LocalPoseConfig`にまとめている。

| 項目 | 値・意味 |
|---|---|
| 初期の経路方向の不確実性 | 選択ノード担当セルの事後分布とセル内分散から計算 |
| 初期の経路横方向の標準偏差 | 0.5m |
| 初期方向の標準偏差 | 20度 |
| 並進の過程ノイズ | `odom_noise`。既定0.1m/√m、車体XYへ等方的に加える |
| 走行による方向の過程ノイズ | 0.05rad/√m |
| 旋回による方向の過程ノイズ | 0.05rad/√rad |

XY補正時はXYの標準偏差を`max(lateral_std, goal_eval_position_tolerance)`（既定0.5m）と仮定する。
元のXYとyawの相関はクリアするが、yawの平均・分散は維持し、誤差ゼロにはしない。
同じbeliefを独立した観測としてもう一度掛け合わせるKalman更新ではない。
これらは実機で同定した誤差モデルではない。ROS共分散はx・y・yawを0・1・5番へ格納する平面モデル。
相対ゴールの共分散にも座標変換のヤコビアンを適用するが、地図の記録姿勢の誤差、
ノード誤認識の確率はこの共分散に含めていない。長距離の無補正追跡や大きな方向分散では
Gaussianの一次近似も不十分になる。

## 診断の読み方

| フィールド | 意味 |
|---|---|
| `header.stamp`, `subgoal_idx` | 入力画像時刻と実際に選択したゴール画像ID |
| `anchor_node_idx`, `anchor_timestamp` | 初回または最後に通常arrivalでXY基準を合わせたノード・画像時刻。向きの基準更新を意味しない |
| `anchor_node_probability` | その基準更新時のノードの全体beliefに対する確率。共分散とは別の曖昧さ |
| `status` | 補正した画像では`ARRIVAL_POSITION_RESET`、以後は`TRACKING_POSITION_ANCHOR`。初期基準のみでは`TRACKING_ASSUMED_ANCHOR` |
| `diagnostic_only`, `heading_reference_assumed` | 常にtrue。制御用の確定姿勢ではない |
| `odometry_continuous` | アンカーからのodom連続性が保たれているか。自己位置の正しさを保証する値ではない |
| `pose_timestamp` | 最後に局所姿勢を更新できた時刻 |
| `anchor_epoch`, `odometry_epoch` | 初期と現在のodom世代 |
| `belief_position_disagreement`, `belief_yaw_disagreement` | 局所推定と1D最尤セルの位置・向きの差。再度尤度として掛けない |

odomの欠損時間だけでは局所追跡を無効にしない。同じ座標系で妥当な端点間の並進・向きを保持する。
欠損中の画像時刻を無理に補間することはなく、新しい有効なodomが揃ってから追跡を更新する。
位置ジャンプ・frame変更・時刻巻き戻り・同一時刻で異なる姿勢が届いた場合は、局所追跡を無効にして最後の姿勢を保持する。
以後のメッセージは`odometry_continuous=false`となり、`pose_timestamp`も古いままになる。
現在時刻の確かな姿勢として解釈しない。自動再同期を入れていないため、再開にはナビゲーションを再起動する。
既存の1Dフィルタとナビゲーションは、従来のodom世代変更処理で動き続ける。

`goal_pose`の座標変換が正しいことと、位置合わせや地図が正しいことは別。
画像ID・時刻・相対方向・横ずれ・不確実性をbag上で照合する。
向きまで合わせる方式と補正なしは検証用スクリプトで比較し、起動オプションには追加していない。
現時点ではXY補正の試験実装であり、制御利用・本採用の判断は検証結果を踏まえて行う。

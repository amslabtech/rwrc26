# ゴール位置・向きの評価とサブゴール選択（第2段階）

`motion_model:=odometry` のとき、選択した画像ゴールに対する診断を
`/toponav/goal_evaluation` (`placenav_viz_msgs/GoalEvaluation`) へ配信する。
`nearest` / `passage` / `arrival`で同じ評価を使う。
`passage`は記録位置の通過、`arrival`は目標ノードへのbeliefの支持と記録位置への接近で中間画像を切り替える。
実機・debug launchの既定は`subgoal_update:=arrival local_pose:=true`。
通常到達時の局所XY補正を使い、相対ゴールポーズを出力する構成としている。
VPRの観測用ownerは中点方式のまま。局所2D推定は[第3段階の診断](LOCAL_POSE.md)として追加した。
局所2D診断には通常arrival時のXY基準補正を追加した。向きは維持し、この補正をbeliefへ戻さない。
制御への相対ゴール接続は未実装。

## デバッグで最初に見る項目

このトピックは、VPRとオドメトリを反映した**更新後のbeliefから、画像ゴールをどう評価・選択したか**を示す。
スコアは0〜1で、画像の類似度そのものではない。実際のXYゴール姿勢は
`local_pose:=true`で配信する別トピック`/toponav/local_goal_pose`を参照する。

普段は次の項目を組み合わせて確認する。

| 項目 | 意味・確認すること |
|---|---|
| `current_node_idx` | 自己位置推定で選んだ現在ノードID。撮影位置への到達済みを意味しない |
| `subgoal_idx` | **現在モデルへ渡しているゴール画像のID** |
| `selection_status` | ゴールを保持・変更した理由。[状態一覧](#selection_statusの読み方)を参照 |
| `decision_goal_idx` | **今回、切り替え・最終到達の判定対象にしたゴールID** |
| `decision_score` | **実際の判定に使用したスコア**。意味はモードによって変わる |
| `neighborhood_mass` | 出力中ゴールの前後近傍に集まったbeliefの量 |
| `confirmation_count` | 通常判定の連続成立回数。成立・切り替え時には0へ戻る |

別ターミナルで、重要項目だけ表示する例：

```bash
source /home/user/ws/install/setup.bash

rostopic echo /toponav/goal_evaluation |
  grep --line-buffered -E '^(current_node_idx|subgoal_idx|selection_status|decision_goal_idx|decision_score|confirmation_count|neighborhood_mass|recovery_candidate_idx|recovery_confirmation_count):|^---$'
```

### 切り替え前後で評価対象が変わる

**`decision_*`と`recovery_*`は判定対象のゴール、通常の評価スコアは出力中の`subgoal_idx`に対応する。**
通常切り替え・救済が発生したメッセージでは、旧ゴールと新ゴールの情報が同居する。
切り替えがない場合は、どちらも同じゴールを評価している。

次はNode 11への到達条件が成立し、Node 12へ画像を切り替えた説明用の例：

```yaml
current_node_idx: 11
subgoal_idx: 12
selection_status: "ADVANCED_ARRIVAL"
decision_goal_idx: 11
decision_score: 0.62
position_score: 0.01
confirmation_count: 0
```

Node 11を評価した`decision_score`が0.62となり、次の画像へ進んでいる。
`position_score`が0.01でも異常ではない。これは**新ゴールNode 12**の評価で、まだ到達していないため低くてよい。
同じ理由で、切り替え直後の`neighborhood_mass`を、旧ゴールの判定条件の検証に使わない。
通常条件を追う際は、切り替え前のメッセージも併せて記録する。

### モード別の判定値とYAMLの対応

以下の距離・しきい値・確認回数は同梱YAMLの初期値。変更した場合は、有効な設定で読み替える。
位置はすべて**経路上の1次元進捗**で、XY平面の直線距離ではない。

| 判定対象 | `decision_score`の意味 | 範囲を決める設定 | 必要な新規画像数 |
|---|---|---|---|
| 中間`arrival` | 目標ノードに所属するセルについて、記録位置±0.5mにいるbeliefの量を集計 | `goal_eval_position_tolerance` | 1回 |
| 中間`passage` | 記録位置から前方2m以内にいるbeliefの量 | `goal_eval_neighborhood_radius` | `pass_confirmations`（初期値2）回連続 |
| 全odomモードの最終到達 | 最終ノードの記録位置±0.5mにいるbeliefの量。ノード所属や方向は要求しない | `final_goal_tolerance` | `pass_confirmations`（初期値2）回連続 |
| 中間`nearest` | 値は計算するが、画像切り替えの条件には使わない | 現在ノード＋`lookahead`で画像を選択 | 連続確認なし |

通常の到達・通過判定には、判定対象ゴールについて次の両方が必要：

- `decision_score >= pass_threshold`（初期値0.5）。
- `neighborhood_mass >= pass_local_mass`（初期値0.8）。近傍の半径は`goal_eval_neighborhood_radius`（初期値2m）。

**`arrival_score`は位置＋方向の診断値で、現在の中間arrivalの判定値ではない。**
実際の判定には`decision_score`を見る。現在の到達・通過・最終到達で、方向診断スコアを必須条件にはしない。

### よくある見え方と確認先

| 見え方 | 確認すること |
|---|---|
| ゴールが進まず`WAITING`が続く | `decision_score`が`pass_threshold`へ届いているか |
| `REMOTE_BELIEF`が続く | `neighborhood_mass`が`pass_local_mass`へ届いているか。ゴール近傍以外にbeliefがある |
| `position_score`は高いが、中間arrivalの`decision_score`が低い | 近隣ノードにもbeliefが分かれていないか。`position_score`はノード所属を問わず、arrivalの判定は目標ノードへの支持も要求する |
| 確認回数が1から0へ戻る | 条件が外れた場合と、切り替え・救済が成立した場合の両方がある。`selection_status`とIDを併せて確認する |
| ゴールIDが複数進む・戻る | `RECOVERED_FORWARD` / `RECOVERED_BACKWARD`か確認する。救済先候補と前後のスコアも見る |
| `arrival_score`だけ低い | 方向診断が低い可能性がある。実際のarrival判定は`decision_score`で確認する |
| 評価が更新されない | bagの一時停止、画像入力、画像時刻に対応するodomの待機を確認する。最終到達後も新しい評価は配信しない |

`GOAL_REACHED`は到達判定の成立を示す。実際の速度出力がゼロかは、TEBの状態・速度トピックで別途確認する。
このトピックはlatchedではないため、最終到達後に購読を始めても過去の`GOAL_REACHED`は再配信されない。

## 実行

新しいメッセージと起動スクリプトをビルドする。既存の`Viz`と`LocalizationDiagnostics`の定義は変更していない。
`navigate_whill.sh`は、ワークスペースにビルドしたメッセージをDockerイメージ内の旧版より優先する。
これにより、ビルド前から開いていたシェルでも新しいメッセージで起動できる。
今回のarrival中間判定の変更ではメッセージ定義を変更していない。
既にビルド済みなら、ナビゲーションの再起動で反映される。

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
  motion_model:=odometry subgoal_update:=arrival \
  bag:=/home/user/bagfiles/tsukuba/20261107_111825_777874.bag \
  bag_pause:=true
```

別ターミナルも`source /home/user/ws/install/setup.bash`してから確認する。
メッセージ定義を更新したため、実行中のナビゲーション・購読ノードは再起動する。
再生停止中は、新しい評価も出ない。

```bash
rostopic echo /toponav/goal_evaluation
```

`navigation.launch`、`navigation_debug.launch`、`navigate_whill.sh`から設定できる。
既定のrosbag記録トピックにも追加している。

実機launchは[placenav.yaml](../src/rwrc26_bringup/config/placenav.yaml)、
debug launchは[placenav_dev.yaml](../src/rwrc26_bringup/config/placenav_dev.yaml)を読み込む。
数値launch引数を省略するとYAMLを使用し、明示した引数はYAMLより優先する。
`placenav_config:=/絶対パス/custom.yaml`で別ファイルも指定できる。変更後はlaunchを再起動する。

| YAMLのキー | launch引数 / 単独CLI引数 | 初期値 |
|---|---|---|
| `goal_eval_position_tolerance` | `goal_eval_position_tolerance` / `--goal-eval-position-tolerance` | 0.5 m |
| `goal_eval_yaw_tolerance_deg` | `goal_eval_yaw_tolerance` / `--goal-eval-yaw-tolerance` | YAMLでは20.0度、引数ではrad |
| `goal_eval_neighborhood_radius` | `goal_eval_neighborhood_radius` / `--goal-eval-neighborhood-radius` | 2.0 m |

YAMLの角度だけは読みやすい度単位とし、runnerでradへ変換する。既存の引数はradのまま維持する。
20度は方向診断用の暫定許容幅で、現在のarrival・passage・最終到達の必須条件ではない。
`navigate_whill.sh`単独ではこれらのYAMLは読み込まず、従来のCLIと既定値を使用する。

位置は経路進捗`s`で評価する。許容距離・半径は正、角度は0より大きくpi以下、
周辺半径は位置許容距離以上とする。既定値は実験の出発点であり、正確な物理到達を保証しない。

## 評価内容

各セルの進捗を`s_i`、全体に対する確率を`b_i`、目標ノードの記録進捗を`s_g`とする。
方向は既存モデルの`yaw + heading_offset`と、目標の記録yawの最短角度差を使う。

| フィールド | 意味 |
|---|---|
| `query_timestamp` | 評価に使った入力画像の時刻。bag・画像・他の診断との照合に使う |
| `current_node_idx` | ノード別beliefの合計が最大のノードID |
| `subgoal_idx` | 評価と同じタイミングでモデルへ渡す画像ゴールID |
| `subgoal_update` | 実行中の選択モード：`nearest` / `passage` / `arrival` |
| `position_tolerance` | 位置診断と中間arrivalで使う進捗の許容幅 [m]。最終判定の`final_goal_tolerance`とは別 |
| `yaw_tolerance` | 方向診断で使う許容角 [rad]。YAMLの`goal_eval_yaw_tolerance_deg`は度だが、配信値はrad |
| `neighborhood_radius` | 近傍評価の進捗半径 [m] |
| `goal_s` | 出力中ゴールの記録進捗 [m]。Node 0が0で、記録ノード間のXY距離を累積した座標 |
| `goal_yaw` | 出力中ゴールの記録方向 [rad] |
| `position_score` | `abs(s_i-s_g) <= position_tolerance`の確率合計 |
| `arrival_score` | 上記に加え、方向差が`yaw_tolerance`以下の確率合計。方向診断であり、中間arrivalの判定値とは異なる |
| `neighborhood_mass` | `abs(s_i-s_g) <= neighborhood_radius`の確率合計 |
| `local_passage_score` | `0 <= s_i-s_g <= neighborhood_radius`の確率合計 |
| `local_passage_heading_score` | 上記に加え、方向差が許容角度内の確率合計 |
| `map_s` | 最も確率が高いセルの進捗 [m]。`goal_s - map_s`は前後関係の目安で、正なら推定位置はゴールの手前 |
| `map_yaw` | 最尤セルの推定方向 [rad]。実測の絶対方向ではない |
| `map_yaw_error` | 最尤セルの推定方向からゴールの記録方向を引いた最短角度差 [rad] |
| `q10_s`, `q50_s`, `q90_s` | セル平均位置に確率を付けた分位点。セル内分散は含まない。複数の位置候補を一つの位置へ確定する値ではない |
| `heading_reference_assumed` | 常にtrue。初期化・odom基準取り直し時、各候補の方向を地図方向に合わせる仮定がある |
| `odometry_epoch` | frame・時刻・姿勢の不連続による基準取り直しの世代。単なる入力間隔の空白では更新しない |
| `boundary_clipped_mass` | 旧ツール用のフィールド。範囲外をclipしない現モデルでは0 |
| `endpoint_mass` | 始点・終点に隣接する内側セルの確率合計。最終到達判定そのものではない |
| `outside_mass` | 始点より前、または終点より先にいる確率。セル内ガウス分布の裾も積分する。経路から横へ離れた確率ではない |
| `zero_node_fraction` | 確率が厳密に0のノードの割合。高いだけで異常とは限らない |

`query_timestamp`は現在画像の時刻。`subgoal_idx`はこの評価と同じ画像ゴールを示す。
周辺だけで確率を再正規化しない。遠方にある確率は、到達スコアに混入しない。
`local_passage_heading_score`は診断値であり、passageで方向一致を必須にしない。
`map_s`のMAPは「最大事後確率」の意味で、ROSの`map`フレームとは別。
`current_node_idx`はノード単位の確率合計、`map_s`はセル単位の最大値なので、同じノードを指すとは限らない。
最尤セルは多峰性のbeliefで飛び得る。`q90_s - q10_s`は分布の広がりの目安だが、
セル内分散を含まず、途中の領域へ均等に確率があるという意味でもない。

## 切り替えと救済

初回は全探索の結果から従来と同じ画像をすぐ出す。`nearest`の中間画像選択も従来どおり。
`passage` / `arrival`は`lookahead=1`でゴールを保持し、新規画像ごとに次の条件を確認する。

- `passage`：`local_passage_score >= pass_threshold`。目標から前方2m以内の全体確率を使う。
- 中間`arrival`：目標ノードのownerに属し、記録進捗±0.5mにも入るbeliefの合計が`pass_threshold`以上。
  セル内の位置分散も積分する。方向差20度は中間判定の必須条件にしない。
- 両モード共通：目標±2mの`neighborhood_mass >= pass_local_mass`（既定0.8）。
- `passage`は既定2画像、中間`arrival`は1画像で次へ一つ進める。
  passageの確認中に条件が外れたら回数をクリアする。救済と最終停止は既定2画像のまま。

中間arrivalのスコアは`sum(b_i * I(owner_i == goal) * near_weight_i)`。
位置とノード所属の周辺確率を独立と仮定して掛ける方式ではない。
ノード間隔が広い場合も、目標が最寄りになっただけでは記録位置±0.5mの条件を満たさず、
中点で切り替える方式には戻らない。密集した旋回ノードは、その画像ノードへの支持で区別する。
初期向きを仮定したodom角度で到達を拒否することを避けるが、正確な撮影姿勢への一致を保証しない。
beliefはこの判定でリセットしない。`local_pose:=true`の診断は通常arrivalでXY基準を更新する。
確定した救済でも、beliefと局所位置が大きく矛盾する場合だけXY基準を更新する。向きは維持する。
条件と制限は[局所姿勢の説明](LOCAL_POSE.md)を参照。

実際の中間arrivalの判定には`decision_goal_idx`と`decision_score`を参照する。
`arrival_score`は従来の位置＋方向の診断値として残し、メッセージの互換性を保つ。

連続確認は一時的な変動を抑えるための処理であり、似た連続画像が独立した証拠になるという保証ではない。

遠方へのbelief移動を通常の通過として数えない。保持ゴールから前方2m以上に0.8以上の確率、
または後方4m以上に0.8以上の確率があり、推定現在ノード±2mにも0.8以上の確率がある場合に救済する。
同じ救済先と方向を2新規画像で確認して、`min(現在ノード+1, 最終ノード)`へ変更する。
短い戻りを抑えるため、後方の許容距離を前方より大きくしている。いずれも経路上のメートル単位。
大きな誤認識が継続すれば救済も誤るため、独立した位置真値による保証ではない。

`/toponav/goal_evaluation`で理由も確認できる。

| フィールド | 意味 |
|---|---|
| `selection_status` | `ADVANCED_PASSAGE` / `ADVANCED_ARRIVAL`は通常切り替え。`RECOVERED_FORWARD` / `RECOVERED_BACKWARD`は救済 |
| `decision_goal_idx`, `decision_score` | 切り替え前に評価したゴールと実際の判定スコア。中間arrivalは目標ownerと位置範囲の同時成立質量。通常のスコア群は切り替え後の`subgoal_idx`に対応する |
| `confirmation_count` | 現在の通常判定の連続回数。成立・切り替え時は0へ戻る |
| `recovery_candidate_idx`, `recovery_confirmation_count` | 確認中の救済先（未設定は-1）と回数 |
| `recovery_forward_score` | 判定前ゴールの進捗＋`neighborhood_radius`より前方にある全体確率。初期値ではゴールより前方2m以上 |
| `recovery_backward_score` | 判定前ゴールの進捗−`goal_recovery_distance`より後方にある全体確率。初期値ではゴールより後方4m以上 |

救済や画像変更だけでは到達とはみなさず、odom入力やbeliefをリセットしない。
局所2D診断のXY補正は通常arrival時と、確定した救済でbeliefと局所位置が大きく矛盾する場合に行う。
向きは維持し、この補正を自己位置beliefへ戻さない。
救済は前方・後方スコアだけでは成立せず、推定現在ノードの近傍への集中と、
同じ救済先・方向の連続確認も必要。成立時は救済候補を-1、確認回数を0へ戻す。

### selection_statusの読み方

| 値 | 意味 |
|---|---|
| `INITIAL` | 初回画像から最初のゴールを選んだ。自己位置推定の安定待ちではない |
| `NEAREST` | 現在ノード＋lookaheadで中間画像を選んだ |
| `REMOTE_BELIEF` | 判定対象ゴールの近傍に必要な確率が集まっていない |
| `WAITING` | 近傍の確率は十分だが、到達／通過の判定スコアがしきい値未満 |
| `CONFIRMING` | 通常条件が成立し、必要な連続回数を確認中 |
| `ADVANCED_ARRIVAL` | 中間arrivalが成立し、次の画像へ一つ進んだ |
| `ADVANCED_PASSAGE` | 中間passageが成立し、次の画像へ一つ進んだ |
| `RECOVERY_PENDING_FORWARD` / `RECOVERY_PENDING_BACKWARD` | 前方／後方への救済候補を連続確認中 |
| `RECOVERED_FORWARD` / `RECOVERED_BACKWARD` | 救済で前方／後方の画像へ変更した。到達成立とは別 |
| `GOAL_REACHED` | 最終到達条件が成立した |

同じ更新で救済の確認へ進むと、通常判定の状態より`RECOVERY_PENDING_*` / `RECOVERED_*`が優先して表示される。

定義と実装の参照先：

- [GoalEvaluation.msg](../src/placenav_viz_msgs/src/placenav_viz_msgs/msg/GoalEvaluation.msg)：メッセージの全フィールド。
- [goal_evaluation.py](../src/placenav/goal_evaluation.py)：画像ゴールに対する評価スコア。
- [subgoal_selector.py](../src/placenav/subgoal_selector.py)：実際の到達・通過・救済判定。
- [navigate.py](../src/placenav/navigate.py)：切り替え後の評価と判定結果を組み合わせて配信する箇所。

## 更新頻度依存・端点処理の修正

セルは連続した進捗の平均と分散を保持する。移動するたびに固定位置へ確率を配分し直す
数値拡散を避け、セルへ集約するときも平均・分散を保存する。
予測は3点の近似とモーメントの集約を使い、直線・観測更新なしの同じ移動について
更新分割数を変えても平均・総分散が一致する。曲線の幾何近似や観測頻度の影響まで
すべて消えるという保証ではない。

過程ノイズの意味は`variance += odom_noise**2 * travelled_distance`へ変更した。
既定の0.1は1m移動で標準偏差0.1mとなる係数で、以前の「各更新の移動量に比例する標準偏差」
とは異なる。横方向への迂回や往復も実際の並進距離として不確実性へ加える。
係数の実機での妥当性は別途検証する必要がある。

始点・終点の外にはそれぞれ1個の状態を持ち、出た先の平均位置・分散を保持する。
地図全体を2D/3Dグリッドへ展開する変更ではない。逆向きに動けば地図内へ復帰できる。
範囲外のVPR尤度は端のノードに関連付けるが、到達スコアは実際の推定位置の範囲を積分する。
範囲外の場所の見え方を新しく学習したわけではない。

全odomモードの最終到達は`final_goal_tolerance`以内の確率が`pass_threshold`以上、
目標近傍の確率が`pass_local_mass`以上であることを連続確認する。
方向一致は全モードで要求しない。エリアは現在の1D進捗beliefにおける経路上の範囲であり、
XY平面の円形領域を直接評価する判定ではない。`nearest`も最終ノードIDだけでは終了しない。
成立後は停止を保持し、救済や遅延Pathで再開しない。

## 解釈上の制限と整合性確認

- 1次元進捗から求めた近似スコアであり、横ずれや方向の共分散を含む3次元の到達確率ではない。
- 同じ`owner`内には同じVPR尤度を掛ける。細かいセルがあっても、その内部を画像だけで精密に識別できない。
- ノード確率を担当区間へ均等に配るため、不等間隔ノードによる初期の確率密度の違いは残る。
- 初回画像を初期化と観測更新に二重使用しないよう修正した。初期化の戻り値が最初の観測結果となる。
  odometry APIでは`initialize_model(image, image_time_odometry)`で最初の積分基準も設定する。
- 位置の区間確率はセル内のガウス近似を積分する。方向は各セルの代表値であり、方向の共分散は未保持。
- VPRの観測更新はセル平均の所属ノードに対応する尤度を使う。広い分布や旋回には近似誤差が残る。
- 似た画像の反復による過信や、確率0の候補への復帰困難も既知の検証対象。

## 検証

```bash
cd /home/user/ws/src/PlaceNav
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py' -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s src/rwrc26_bringup/test -p 'test_*.py' -v
```

単体検証は、ノード密度に依存しない物理範囲、方向差のwrap、遠方の確率、
評価による状態不変性、範囲外の保持、初回画像の一回処理、CLI/launch引数、
新規画像での確認、遠方の多峰性、後方ヒステリシス、到達失敗時の前方救済、最終停止を含む。
bag検証では、保存した実VPR特徴による全区間評価と、実モデル・ROSによる区間再生を区別する。
地図作成元bagの記録odomとの比較は、独立した位置真値による精度検証ではない。

全体の小さな事前分布を混ぜる試作も比較したが、別日bagで遠方の似た画像へ移る副作用が出たため採用していない。
自己位置beliefそのものの長い遅れ・誤認識は、このサブゴール選択だけでは解決しない。

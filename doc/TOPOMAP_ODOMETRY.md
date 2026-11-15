# オドメトリ付きトポロジカルマップ

従来の画像マップと、新しいノード間相対姿勢付きマップを、同じ生成スクリプトで切り替える。
保存した地図上の位置関係とライブodomを使う、ナビゲーション側の移動更新にも対応する。

## 生成モードと距離指定

コンテナ内の `/home/user/ws/src/PlaceNav/src` で実行する。

```bash
# 既定のオドメトリ付きモード。直線区間のノード間隔を2.0mにする。
./create_topomap_whill.sh tsukuba_odom_2.0 \
  --straight-distance 2.0 --odom-topic /odom
```

直線距離は、最後に保存した画像の位置と現在位置の
`sqrt((x2-x1)^2 + (y2-y1)^2)`。走行した道のりの累積長ではない。
画像の到着間隔などにより、距離がしきい値を超えた最初の処理タイミングで保存するため、
厳密に2.000m間隔になるわけではない。
旋回時には従来の適応的保存を維持するため、2mより短い間隔でも保存される。

```bash
# 第2位置引数も引き続き有効。秒数であり、距離ではない。出力は既定のodom形式。
./create_topomap_whill.sh route_with_period 2.0

# 従来形式の出力で、距離だけ2mに指定することもできる。
./create_topomap_whill.sh old_route_new_name --map-mode legacy --straight-distance 2.0
```

| オプション | 既定値・意味 |
|---|---|
| `--map-mode` | `odometry`。`legacy` で従来形式を選択 |
| `--straight-distance` | 3.0m。直線区間の保存距離 |
| `--sampling-mode` | シェルでは `adaptive`。`time` も指定可能 |
| 第2位置引数／`--dt` | 3.0秒。`time` モードの時間間隔 |
| `--odom-topic` | ロボット設定の値。WHILLではホイール由来の `/odom` を使用する想定 |
| `--odom-max-gap` | 0.2秒。画像を挟む2つのオドメトリの最大時間間隔 |
| `--max-node-distance` | 新モードの `adaptive` では `2*straight-distance` m。これを超えるノード間接続は保存前にエラー停止。`time` では明示指定時のみ適用 |
| `--idle-timeout` | 無画像時の終了待ち秒数。新モードでは0＝Ctrl+Cまで待機。従来モードは既定で `2*dt` |
| `--topomap_directory` | 既定は `src/topomaps/images` |

新モードは既存の同名ディレクトリを上書きしない。別のルート名を指定する。
従来モードの同名マップ置き換え動作は維持している。

## 指定bagから作る例

以下はコンテナ内。先にROS masterを起動する。

```bash
roscore
```

別ターミナルで、生成ノードより前にsim timeを有効にする。

```bash
rosparam set /use_sim_time true
rosrun image_transport republish compressed in:=/usb_cam/image_raw raw out:=/usb_cam/image_raw
```

別ターミナルで上記の `create_topomap_whill.sh` を起動し、
最後にさらに別ターミナルで入力だけを再生する。

```bash
rosbag play --clock --delay=3 \
  /home/user/bagfiles/tsukuba/tsukuba.bag \
  --topics /odom /usb_cam/image_raw/compressed
```

bag再生終了後、生成ノードのターミナルでCtrl+Cを押す。
JSONは各ノード保存時に更新済みで、終了時に従来の `traj_data.pkl` も保存される。
深度推定・TEB・車体ドライバの起動は不要。
実機のライブ入力で生成するときは、生成ノードを起動する前に `rosparam set /use_sim_time false` に戻す。

## 保存内容と座標系

```text
tsukuba_odom_2.0/
  0.jpg
  1.jpg
  ...
  traj_data.pkl
  node_odometry.json
```

JSONは `schema_version: 1`。共通情報として入力トピック、オドメトリの
`frame_id` と `child_frame_id`、保存条件、単位（m、rad）を記録する。

- `nodes`: 連番ID、画像名、画像のROS時刻、画像時刻での `pose: {x, y, yaw}`、
  補間に使った前後のオドメトリ時刻。
- `edges`: 隣接する保存ノードの `from`、`to`、`dx`、`dy`、`dyaw`、直線距離 `distance`。
- 時刻は精度を落とさない `{secs, nsecs}` 形式。
- `pose` は入力オドメトリの親座標系での座標。先頭ノードをゼロには変換しない。
- `dx,dy` は出発ノードの車体座標系。前向きが+x、左向きが+y。
  `dyaw` は出発姿勢からの角度差を±πの範囲に正規化した値。

例えばid:1→id:2の相対位置は、`edges` の `from: 1, to: 2` に保存する。
`nodes[].pose` は各画像のodom座標系での姿勢であり、相対位置とは別の補助情報。
`odom_before_stamp` と `odom_after_stamp` は、その画像の `stamp` を挟む
2つのodomメッセージの時刻で、ノード間の始点・終点の時刻ではない。
ノード間の始点・終点の時刻は、`from` と `to` に対応する各ノードの `stamp` を参照する。

画像時刻を挟むオドメトリでx,yを線形補間し、yawは角度の折り返しを考慮して補間する。
同じ時刻のオドメトリがあれば、その値を使用する。外挿はしない。
前後データ不足・大きすぎる時間間隔ではノードを保存しない。
入力復帰時の画像が最後の保存ノードから `--max-node-distance` より遠ければ、
その画像・接続を追加する前に異常終了し、保存済みの区間だけを残す。
例えば直線間隔2mでは既定上限4m。これは離散的な画像取得による超過を許すための安全上限であり、
すべての区間が正確に2mになる保証ではない。より厳しい運用では `--max-node-distance 2.2` などを指定する。
異常終了した出力はルート途中までの地図であり、全ルート完成として扱わない。
欠けた区間の画像を複製したり、架空の画像ノードを生成したりはしない。
座標系の変更やオドメトリ時刻の巻き戻りがあれば、座標を混ぜず生成を停止する。
同じ座標系のまま位置だけをリセットする入力は、利用者側で避け、新しいマップとして再開する。

画像書き込みの成功後にJSONを更新する。一時ファイルへ書いてから置換するため、
更新途中のJSONを通常の読み手が読むことを防ぐ。
新モードの `traj_data.pkl` も保存ノード間の相対姿勢と整合させる。
従来モードは既存形式を維持し、JSONは追加しない。

## オドメトリ入力の変更

```bash
./create_topomap_whill.sh route_from_vo --map-mode odometry \
  --straight-distance 2.0 --odom-topic /vo/odom
```

入力は `nav_msgs/Odometry`。トピック名だけでホイール／VOを判別する処理はない。
VOを使用する場合も同じ車体基準の連続したオドメトリを用意する。
カメラ基準のVOから車体姿勢への変換やVO推定器自体の実装は対象外。
この記録はオドメトリに対する整合性を持つもので、実際の位置精度や累積誤差の改善を保証しない。

## ナビゲーションの移動更新にodomを使う

```bash
./navigate_whill.sh --topomap-dir whill_odometry_tsukuba_2.0 \
  --wp-model nomad --pr-model salad
```

統合launch・bagデバッグからも同じモードを選択できる。

```bash
roslaunch rwrc26_bringup navigation_debug.launch \
  topomap_dir:=whill_odometry_tsukuba_2.0 \
  bag:=/home/user/bagfiles/tsukuba/tsukuba.bag bag_pause:=false
```

既定は `motion_model:=odometry`。CLIの既定も `place_recognition + bayesian + odometry`。
`motion_model:=legacy subgoal_update:=nearest local_pose:=false`
（単独CLIは `--motion-model legacy`）で従来の固定ノード窓による移動更新を選択する。
`sliding_window` や `temporal_distance` を使う場合も `--motion-model legacy` を明示する。
launchの既定地図は `whill_odometry_tsukuba_3.5`。別の地図は `topomap_dir` で指定する。
地図画像と対応する `node_odometry.json` が必要で、不足時は起動エラーになる。
VPRの観測尤度、移動更新、観測更新用の中点境界 `owner` は共通。
画像サブゴールの選択は、従来方式と下記の保持方式を切り替えられる。

| launch引数 | 直接起動の引数 | 意味 |
|---|---|---|
| `motion_model:=odometry` | `--motion-model odometry` | ライブodomによる予測（既定） |
| `odom_topic:=/odom` | `--odom-topic /odom` | 車体基準の `nav_msgs/Odometry`。直接起動では省略時にrobot設定を使用 |
| `odom_max_gap:=0.2` | `--odom-max-gap 0.2` | 画像を挟むodomの最大間隔（秒） |
| `odom_noise:=0.1` | `--odom-noise 0.1` | 経路方向の過程ノイズ係数。走行距離`d` mで分散`odom_noise²*d` m²を加える。1mで標準偏差0.1m、更新分割数に依存しない意味へ変更 |

画像時刻へ補間したodom同士の相対移動を、一度ずつ反映する。
地図は各ノードの `pose` から経路の位置関係を読み、経路に沿った距離座標へ展開する。
内部では最大0.1m、最短ノード間隔の半分以下の間隔で途中位置も保持するため、
小さな移動を積み重ねられる。各ノードへの確率を集約して従来と同じVPR観測更新を行う。
サブゴールの切替では移動量をリセットしない。
各セルには連続した位置の平均・分散を保持し、固定位置への反復丸めによる拡散を避ける。
始点より前・終点より先の仮説は各1個の範囲外状態へ集約し、その位置・分散を失わない。
区間確率はセル内のガウス近似を積分する。

並進はその時点の車体向きから経路の進行方向へ投影する。
画像間のyaw変化も保持するため、その場旋回や横への移動を単純な前進距離には加算しない。
これは経路付近で追跡するモデルであり、任意の迂回路を地図化したり、
経路外の状態を推定する仕組みではない。長時間のodom誤差や異なる視点でのVPR誤認は残り得る。

odomは地図作成時と原点・親座標系が異なっていてもよいが、`child_frame_id` は地図と同じ車体基準が必要。
VOも同じ条件の連続した入力を用意すればトピックを変更して使用できる。
画像時刻の前後odomが揃うまでは新しい推定・経路をpublishせず待機する。外挿や従来モードへの自動切替は行わない。
同じ画像時刻の反復はVPR・移動量とも二重反映しない。
同じ座標系で時刻が前へ進み、変位が速度上限と整合するodomは、欠損時間だけでは基準を取り直さない。
欠損前後のodom差分で並進・yaw変化を反映し、地図と車体の向きの補正を保持する。
ただし、画像時刻の補間には引き続き `odom_max_gap` を適用し、欠損区間を無理に補間しない。
時刻巻き戻り・フレーム変更・明らかな並進ジャンプ・同一時刻で異なる姿勢が届いた場合は、
自己位置の確率を残して次の画像でodom基準と向きの補正を取り直す。
ジャンプ判定は隣接odomの変位が `max(3.0, 3*max_v) × 時間差 + 0.05m`を超える場合。
5cmの許容幅で短い時刻間隔の揺れを吸収し、入力odomの移動量自体は変更しない。
同じフレーム内の小さな補正やyawのみのリセットは自動検出できないため、連続したodomを使用する。
欠損中に往復した移動やスリップは端点だけから復元できない。

## 画像サブゴールの切り替えと救済

実機・debug launchは`subgoal_update:=arrival local_pose:=true`が既定。
ノード付近への到達で画像を更新し、局所XY基準を補正する。
`subgoal_update:=nearest`を指定すると、中間画像は従来どおり
`min(closest_node_idx + lookahead, goal_node_idx)`を使う。
`passage` / `arrival`は`motion_model:=odometry`、`lookahead:=1`で選択できる。
単独CLIの既定は従来の`nearest`、局所姿勢出力なしのまま。launchと同じ構成は
`--subgoal-update arrival --local-pose`で指定する。

```bash
roslaunch rwrc26_bringup navigation_debug.launch \
  topomap_dir:=whill_odometry_tsukuba_3.5 \
  subgoal_update:=arrival \
  bag:=/home/user/bagfiles/tsukuba/20261107_111825_777874.bag \
  bag_pause:=true
```

初回は従来と同じ画像をすぐ出す。全探索のbeliefが安定するまで待たない。
以後は画像ゴールを保持し、`passage`は目標の前方近傍へ進んだ確率、
`arrival`は目標ノードに所属し、目標位置の許容範囲にも入るbeliefの質量で次の画像へ進める。
arrivalの中間判定は新規画像1回で行う。観測更新のownerは中点方式のまま。
方向差は診断に残す。最終到達は全モードで経路上の終点エリアを使い、方向一致は要求しない。
近傍はノード数ではなく、経路進捗のメートル単位で評価する。

| launch引数 | 既定値・意味 |
|---|---|
| `subgoal_update` | launchの既定は`arrival`。`nearest` / `passage`も選択可能 |
| `pass_threshold` | `0.5`。全体の事後確率に対する通過・到達しきい値 |
| `pass_confirmations` | `2`。passage・救済・最終到達で確認する連続新規画像数。arrivalの中間判定は1画像 |
| `pass_local_mass` | `0.8`。通常切り替え・最終到達に必要な目標近傍の確率 |
| `goal_eval_position_tolerance` | `0.5 m`。arrivalの記録位置前後の許容幅 |
| `goal_eval_yaw_tolerance` | `0.34906585 rad`。方向診断の許容角度。到達判定には使わない |
| `goal_eval_neighborhood_radius` | `2.0 m`。目標近傍の半径と前方救済の距離 |
| `goal_recovery_distance` | `4.0 m`。後方救済の距離。近傍半径以上にする |
| `goal_recovery_threshold` | `0.8`。救済に必要な全体確率 |
| `final_goal_tolerance` | `0.5 m`。全odomモードの最終目標の許容幅 |

CLIでは引数名のアンダースコアをハイフンにして指定する。
通常切り替えが成立せず目標を通り過ぎた場合は、前方近傍より先に十分な確率があり、
現在ノード近傍にも確率が集中していることを確認して現在ノード＋1へ救済する。
後方も同様だが4mの差を許容し、短い前後変動を抑える。
同じ救済先・方向を連続確認し、最終ノードを上限にする。救済だけで最終到達とはしない。

同じ画像時刻は二重に数えず、条件不成立・odom基準取り直し・時刻巻き戻りでは確認回数をクリアする。
画像切り替え・救済で自己位置beliefやodomをリセットしない。
遠方beliefを通常通過と誤認定しにくくなるが、自己位置誤認識自体の解消機能ではない。
評価トピックの判定理由と注意点は[ゴール共通評価](GOAL_EVALUATION.md)を参照。

全odomモードの最終到達では、目標±`final_goal_tolerance`の確率・メートル近傍の確率を連続確認する。
方向一致は要求しない。現在のエリアは経路進捗の範囲であり、XY平面の円ではない。
途中ノードを最終目標にした場合も、遠く通り越しただけでは終了しない。
`legacy`の到達判定は従来どおり。ここで扱う位置は1D進捗で、横ずれを含む2D距離ではない。

最終到達時は新しい非ゼロwaypointを出さず、到達Boolとゼロwaypointを送る。
ROSノードは存続し、到達通知をlatched publisherで保持する。
TEBは全経路を無効化し、壁時計タイマーでもゼロ速度を送り続ける。
bagの一時停止、遅れたPath、False通知では解除しない。停止処理は全モード共通。
次の走行はlaunch全体を再起動する。

## 検証

ROSをsourceしたコンテナ内で実行する。

```bash
cd /home/user/ws/src/PlaceNav
python3 -B -m unittest discover -s tests -p test_topomap_odometry.py -v
python3 -B -m unittest discover -s tests -p test_odometry_motion.py -v
python3 -B -m unittest discover -s tests -p test_subgoal_selector.py -v
python3 -B tests/check_topomap_odometry_ros.py \
  --output "$(mktemp -d /tmp/topomap-odometry.XXXXXX)"
```

後者は専用のROS master（既定11339）で、実際のシェルと生成ノードを使い、
画像時刻補間、旋回、保存を飛ばした場合の差分、逐次JSON保存、
既存マップ保護、従来CLI／形式、別トピック入力、欠損と座標系変更を確認する。
通常のROS masterや実機の制御トピックには接続しない。

TEBをビルド後、専用masterで終端停止を確認できる。
実機ドライバは起動せず、そのmaster上の `/local_path/cmd_vel` を監視する。

```bash
python3 -B tests/check_navigation_stop_ros.py \
  --bridge /home/user/ws/install/placenav_teb_bridge/lib/placenav_teb_bridge/placenav_teb_bridge_node \
  --output "$(mktemp -d /tmp/navigation-stop.XXXXXX)"
```

TEBのlatest・state_machine・保持パス、直接PD制御で、非ゼロ速度からの停止を確認する。
時計停止、遅れた入力、False通知、到達後に起動したTEBへのlatched通知も確認する。

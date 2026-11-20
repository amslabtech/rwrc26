# OmniVLA-edge（waypointバックエンド）

`--wp-model omnivla_edge`でNoMaDの代わりに選べます。VPR・ベイズフィルタ・サブゴール選択・TEB・STVLは変更なしです。
OmniVLA-edgeは**画像ゴールのみ**のモード（上流のmodality 6）で使い、GPS・言語・衛星地図の入力は使いません。

## 準備と実行

```bash
cd /home/user/ws/src/PlaceNav
python3 src/prepare_waypoint.py --models omnivla_edge   # 重み(HF, MIT)とソースをコミット固定で取得
python3 src/verify_waypoint.py --models omnivla_edge --device cuda
cd src && ./navigate_whill.sh --topomap-dir ROUTE --wp-model omnivla_edge --pr-model salad
```

`model_weights/waypoint/`（重み434 MB＋`sources/OmniVLA`）はGit管理外です。オフライン機へはディレクトリごとコピーします。
重みはSHA256を固定値と照合し、`weights_only=True`で読み込みます。

## 仕様

| 項目 | 値 |
| --- | --- |
| 入力 | 96×96、現在＋過去5フレームと、サブゴール画像（ViNT/NoMaDと同じPILリサイズ） |
| 出力 | 8点、各`[dx, dy, cosθ, sinθ]`。x前方・y左。単位は**0.1 m**（`metric_scale: 0.1`で固定） |
| 学習時の時間刻み | 3 Hz（8点で2.4 s） |
| 推論 | 約10 ms（RTX 3080）、約108Mパラメータ、決定的（`--nomad-seed`は無関係） |
| 制約 | **CUDA必須**。上流モデルが`Tensor.get_device()`を使うためCPUでは動かない。CLIP・GPSは不要 |

未使用の入力は上流モデル内でマスクされるため、ゼロで置き換えて同じ出力になります
（`tests/test_waypoint_backends.py`で、姿勢を使うモードとの対照付きで確認）。

## bag再生での検証（ROSなし・読み取り専用）

`rwrc26_ws/tools/waypoint_bag_replay.py`（PlaceNavリポジトリの外、`rosbags`が必要）：bagの時刻iの画像と、odom上でD m先のフレーム（ゴール）を入力し、予測軌道を実際に走った軌跡と比較します。
`dkan_1syu.bag`と`20261103_7.bag`（datasets内）の741サンプル。結果は`rwrc26_ws/results/omnivla_edge_replay/`。

| ゴール方位が15°以上ずれたサンプル（n=143） | OmniVLA-edge | NoMaD | 常に直進 |
| --- | --- | --- | --- |
| 最終点方向の角度誤差 中央値 / p90 | 12.5° / 58.8° | 10.4° / 83.0° | 27.5° / - |
| 旋回側（左右）の一致率 | 0.93 | 0.89 | - |

- 方向選択はNoMaDと同等で、悪化はない。優位とまでは言えない（サンプル数が少なく、同一走行内のゴールのため両者に楽観的）。
- 文脈フレームを毎フレームにしても3フレームおきにしても差はない（navigate.pyの現状のままでよい）。
- **予測経路が短い**：OmniVLA-edgeの8点の長さは中央値0.54 m、NoMaDは1.54 m（ロボットの実速度は0.36〜0.47 m/s、
  OmniVLAの予測は約0.2 m/s相当）。`/placenav/gnm_path`が短くなるため、TEBの終端到達判定・減速に影響しうる。

## 相対ゴール姿勢（ホイールオドメトリ）を入力に加えた場合（2026-11-18〜20、検証中）

**現時点の結果（採否は未決定）：今の構成とtrackerでは、画像のみ（modality 6）を上回る結果はまだ得られていない。**
`LocalPoseTracker`の相対ゴール姿勢をmodality 5（姿勢＋画像）で入れると、tracker出力のままでは悪化し、
向きを補正すると画像のみと同程度になる。経路からそれた場面、類似度などでの切り替え、XYの置き直し方の変更、
照合手法の変更（SuperPoint＋LightGlue＋5点法）でも、今のところ傾向は同じ。
正しい姿勢を入れれば明確に改善するので、姿勢の精度、特にtrackerの位置（ルートに対する左右）を上げられれば
結果が変わる可能性がある（ルートに対する横位置の直接観測、誤差を含む姿勢での学習など）。
以下の数値はオフライン再生（記録したtracker出力をOmniVLA-edgeへ入力）によるもの。検証スクリプトはリポジトリ外（ローカル）。

姿勢入力は上流の学習コード（`gnm_dataset.py`）と同じ`[x/0.1, y/0.1, cos Δyaw, sin Δyaw]`（x前方・y左・yaw反時計回り）。
OmniVLA-edgeには姿勢の信頼度を渡す入力や調整パラメータはなく、modalityで使う／使わないを選ぶだけ。

### 実機・bag再生で使う（実験用、既定は無効）

```bash
# bringup（parrot、Xavier分割でも同じ。session_runnerが引数を付ける）
roslaunch rwrc26_bringup navigation.launch wp_model:=omnivla_edge local_pose:=true \
    wp_goal_pose:=true heading_correction:=sift_shift ...
# 単独CLI
./navigate_whill.sh --topomap-dir ROUTE --wp-model omnivla_edge --local-pose --wp-goal-pose \
    --heading-correction sift_shift ...
```

- `wp_goal_pose`：`OmniVLAEdgeAdapter`に`goal_pose=(x, y, yaw)`を渡してmodality 5（姿勢＋画像）で推論する。
  渡すのは`/toponav/local_goal_pose`と同じ`LocalPoseTracker.goal(subgoal)`で、初期化済み・odom連続・その画像時刻で
  更新済みのときだけ。それ以外は画像のみ（modality 6）に戻り、切り替わりをログに出す。30 m超は上流と同じく切り詰める。
  `wp_model:=omnivla_edge`と`local_pose:=true`が必須。
- `heading_correction`：サブゴール切り替え（XY基準の置き直し）ごとに、その画像とノード画像の向きの差を別スレッドで
  測り、局所姿勢の向きを「ノードの記録yaw＋差」に置き直す（詳細は`doc/LOCAL_POSE.md`）。補正の適用
  （`LocalPoseTracker.apply_heading`）と推定方法（`placenav/heading_correction.py`の推定器）は分かれていて、
  推定器を1つ登録すれば名前で切り替えられる。現状の`sift_shift`はSIFTの水平ずれと水平画角
  （`robots.yaml`の`camera_hfov_deg: 140`、odomから合わせ込んだ値で校正ではない）による簡易法。
- `tsukuba_lidar.bag` 0〜730 sの再生（`wp_goal_pose`＋`heading_correction:=sift_shift`）：到達98回中90回で補正、
  結果は0.3〜0.4 s遅れで適用、1周期の時間は変わらない（RTX 3080）。下表「画像＋tracker（到達時に向きを画像で補正）」の
  オフライン再現とほぼ同じ結果になった（通過位置の正解で予測方向の誤差4.2° / 13.9°、オフラインは4.1° / 13.6°。
  局所2D剛体変換の正解で4.9° / 17.1°、オフラインは4.7° / 16.8°）。実際に出たwaypointと、記録した姿勢から
  オフラインで計算し直したwaypointは一致した（終点の差の中央値0.0 cm）。Xavierでの処理時間は未計測。

### 模擬誤差（同一走行のbag再生、744サンプル・旋回147）

| 入力 | 全体 中央値 / p90 | 旋回 中央値 / p90 |
| --- | --- | --- |
| 画像のみ | 3.2° / 14.3° | 12.6° / 46.9° |
| 画像＋姿勢（誤差なし） | 2.1° / 8.4° | 5.5° / 24.6° |
| 画像＋姿勢（yaw +20°） | 11.3° / 18.5° | 10.5° / 24.3° |
| 画像＋姿勢（XY 1 m） | 10.1° / 29.4° | 15.6° / 56.6° |
| NoMaD（参考） | 3.4° / 17.3° | 10.0° / 74.9° |

- 姿勢を与えるとゴール画像はほぼ使われない（姿勢のみとの差は約1°）。姿勢の誤差はそのまま経路の誤差になり、
  方位誤差は一定方向の偏りになる。目安は方位誤差10°以下・位置誤差0.5 m以下。

### 実際のtracker出力（別日の走行）

地図は`tsukuba.bag`（2026-10-17）から`whill_odometry_tsukuba_3.5`。評価は`tsukuba_lidar.bag`（2026-11-07）の0〜730 s
（地図のノード0〜162、約300 m、ほぼ直線）を`--local-pose`付きのnavigate.py（`placenav_dev.yaml`と同じ値）で再生した。
正解は、MegaLocで評価走行の各フレームを地図ノードへ対応付け、評価走行自身がそのノードを通った位置とした
（2走行の横ずれは残る）。走行中・サブゴールが前方1 m以上の665サンプル。角度は予測方向の誤差の中央値 / p90。

| 入力 | 1〜2 m先 | 2〜3 m先 | 全体 |
| --- | --- | --- | --- |
| 画像のみ（現状） | 2.9° / 9.0° | 3.4° / 10.9° | 3.6° / 10.6° |
| 画像＋tracker | 7.3° / 20.8° | 6.5° / 23.4° | 7.0° / 23.3° |
| 画像＋tracker（到達時に向きを画像で補正） | 4.5° / 13.9° | 3.7° / 14.7° | 4.1° / 13.6° |
| 画像＋正解姿勢（参考） | - | - | 2.2° / 4.8° |

- 正解の作り方で、画像のみと向き補正trackerの順位が入れ替わる。この表（通過位置の正解）では画像のみ3.6°・向き補正4.1°、
  同じデータで正解だけを次節の局所2D剛体変換に替えると5.9°・4.7°（どちらも中央値）。通過位置の正解は2走行の車線の違いを0とみなし、
  剛体変換の正解は当てはめ残差（中央値0.44 m）を含む。両者の1〜2°の差は、正解の誤差の範囲と考えている。
- trackerの誤差は位置0.85 m・ゴール方向12.8°（中央値）。主因は向きで、リセットされないため2走行のodomの
  ドリフト差が溜まる（143〜162 sで+19〜+26°）。tracker自身のyaw標準偏差も約40°。
- 到達時に向きをノードの記録yawへ合わせるだけでは、ノードごとに3〜10°、曲がり角付近で20°超の誤差を持ち込む。
  画像で測った向きの差を足して補正すると改善し、画像のみと同程度になる（SIFTで4.1°、後述のLightGlueで3.6°。画像のみは3.6°）。
  残る誤差は位置で、到達判定がノードの手前で出るのにXYをノード位置へ置き直すため、ゴールを約0.4 m近く見積もる。
- 補正した姿勢が画像のみより良いのは3 m以上先のゴールだけで、サブゴール＝次のノード（間隔は中央値1.6 m、最大3.6 m）では
  ほとんど起きない。出力の重み付き平均や、距離による切り替えでも差は0.3°程度だった。
- サブゴールが実際には通過済みのサンプルが22%あり、490〜530 sにはサブゴールが保持されたまま
  （`Goal held: belief outside metric goal neighborhood`）trackerはゴールを33〜42 m後方と出していた。
- 活かすには、近距離の位置誤差を減らす（XYの置き直しを手前側にずらす）、姿勢のゴールを数ノード先にする（modality 4）、
  誤差を含む姿勢でファインチューニングする、などが必要。

### 経路からそれた場面（TEBで自律走行したbag）

`placenav_unik3d_teb_tsukuba_2.bag`（2026-10-17、地図の約1時間後、`reference_mode:=state_machine`）の1000〜2550 sを同様に再生した。
回避でルートから外れても正解を置けるよう、ノード通過点への局所的な2D剛体変換（±15 m、Huber）で地図ノードを
評価走行の座標へ写して正解とした（当てはめ残差は中央値0.67 m）。走行中・サブゴールが前方1 m以上の2218サンプル。

| 状況（サンプル数） | 画像のみ | 画像＋tracker（向き補正） | 画像＋正解姿勢 |
| --- | --- | --- | --- |
| ルートからの横ずれ0.3 m未満（702） | 4.1° / 14.7° | 3.9° / 14.5° | 3.1° / 10.7° |
| 横ずれ0.6〜1.0 m（545） | 17.4° / 34.6° | 17.4° / 36.3° | 8.1° / 27.4° |
| 横ずれ1 m以上（408） | 46.9° / 74.6° | 43.3° / 74.8° | 37.7° / 64.9° |
| 全体（2218） | 11.8° / 48.8° | 11.1° / 46.7° | 6.7° / 39.6° |

- それると画像のみは悪化し、正しい姿勢なら改善する。しかしtrackerの誤差も同じ場面で大きくなる（横ずれ1 m以上で
  位置1.67 m・ゴール方向34°）。到達時にXYをルート上のノード位置へ置き直すため、横のずれが消えるのが原因。
- 切り替え（クエリとサブゴールのSALAD・MegaLoc類似度、trackerのゴール方位、向きの差、画像との食い違い）は
  どれも改善1°未満。サンプルごとに良い方を選ぶオラクルでも1.8°。両者はほぼ同じ場面で一緒に失敗する。
- XYの置き直しを前後方向だけにすると、odomの横方向のドリフトが溜まって悪化した（tracker位置誤差1.04→1.73 m）。
  置き直さないとさらに悪い（約8 m）。
- 回避中の到達判定は73回中8回だけで、大きな横ずれの多くはNORMAL状態だった。`reference_mode:=latest`では
  `navigation_state`が常に`TRACKING`なので、回避状態を使う切り替えはできない。

### 照合・姿勢推定の方法による差

到達時の向きの差を、SIFTの横ずれから、SuperPoint＋LightGlue（固定コミット）と5点法（エッセンシャル行列・RANSAC）に
替えた。等距離射影で視線ベクトルへ変換し、焦点距離とカメラの傾きは同じ走行のodomから合わせ込んだ。

- 同じ走行の組でのyaw誤差（中央値 / p90）：SIFT 1.1° / 3.6°、5点法 1.3° / 5.2°（tsukuba）。到達時の推定は全件成功
  （SIFTは94件中87件）。
- 2方式の向きの差は、tsukubaで中央値0.7°・p90 3.4°、TEBで1.6°・5.8°一致した。測定自体は正しく取れていた。
- OmniVLAの結果はほぼ同じ（中央値でtsukuba 4.1°→3.6°、TEB 11.1°→10.9°）。5点法の並進の向きはp90 73°と不安定で、
  到達時（ノードまで0.5 m前後）の位置補正には使えない。
- 推定した焦点距離は撮影日で8%違う（フル解像度で1175 px / 1275 px）。チェッカーボード等での校正はしていない。

### データの注意

- `tsukuba_lidar.bag`は419.69 sと632.23 sのチャンクがLZ4で壊れていて、`rosbag play`は419.7 sで
  `ROSLZ4_DATA_ERROR`により停止する。壊れたチャンク（各約20 ms）を除いて必要なトピックだけをコピーしたbagで再生した。
  カメラは平均約4 Hzで、0.5 s以上の欠けが227か所ある。
- `/usb_cam/camera_info`は空（K・Dとも0）。`config/calibration.yaml`は別のカメラ（1280×720）の値。
  画像から角度を出すときは、odomのyaw変化から換算を合わせた（等距離射影でHFOV約140°）。

## 未検証・残課題

- 2026-11-15に実機（WHILL）で、Jetson AGX Xavier上のnavigate.py（CosPlace＋OmniVLA-edge、1周期 約155 ms）→
  経路アダプタ→TEBの通しで走行できた（`docker/xavier/README.md`）。経路の質は未評価。
  新しい環境では`python3 src/verify_waypoint_ros.py --wp-model omnivla_edge --images DIR`を先に実行すること。
- 障害物回避・衝突は評価していない。TEB＋STVLは外さない。
- 短い参照経路がTEBに与える影響。必要なら`metric_scale`の調整（現在は上流の0.1に固定）や`--target-waypoint-idx`（上流は4）を試す。
- 学習データに車いす型の機体は含まれない（FrodoBots・車載映像など）。WHILLのカメラ高さ・画角との差は未評価。

## C2単独で回避経路を見る（WHILL・速度指令なし）

### 11/19の記録と同じサブゴールで再推論

ROS Noetic・CUDAを使えるPlaceNavコンテナで、リポジトリ直下から実行する。

```bash
source /opt/ros/noetic/setup.bash
python3 src/prepare_waypoint.py --models omnivla_edge  # 重みを配置済みなら省略可
python3 src/preview_omnivla_bag.py \
    bag/1119/omnivla_edge_odometry_nearest_2026-11-19-07-42-51.bag \
    --topomap src/topomaps/images/whill_odometry_ikuta_3.5 \
    --start 760 --duration 10 --output /tmp/c2-person.mp4
```

C2の記録画像と撮影時刻が一致する `/toponav/viz_info` からサブゴールIDを取り出す。
モデル入力はC2の連続6画像と該当ノードの画像。VPRの再推定、D435、TEB、ROS masterは不要。
左が撮影画像、右が前方を上・左方を左とする機体座標の経路（緑＝再推論、青＝bag記録）。
動画は照合できた推論1件につき1フレーム、固定5 fps。画面の時刻はbag先頭からの撮影時刻。
圧縮画像と記録された履歴を使うため、実験当時の入力と完全には一致しない場合がある。
同じ地図・モデルのbagを指定する。記録経路にはモデル名が含まれない。
比較できるのは画像を見た時点の予測であり、実機がその経路で避け切れるかは別途評価する。

### VPRから経路生成まで再実行

`rwrc26_bringup` をビルドしてワークスペースをsourceした環境で、走行系から独立した
ROS masterを使う。以下は11/19屋外の `odometry / nearest` 条件。各端末で同じ
`ROS_MASTER_URI` を指定する。作業コピーを切り替えた後は、コンテナ内で次の2パッケージをビルドする。

```bash
cd /home/user/ws
source /opt/ros/noetic/setup.bash
colcon build --base-paths src/PlaceNav/src/placenav_viz_msgs/src/placenav_viz_msgs src/PlaceNav/src/rwrc26_bringup \
    --packages-select placenav_viz_msgs rwrc26_bringup --symlink-install
source install/setup.bash
cd src/PlaceNav
```

```bash
# 端末1
export ROS_MASTER_URI=http://localhost:11329
roscore -p 11329

# 端末2
export ROS_MASTER_URI=http://localhost:11329
roslaunch rwrc26_bringup omnivla_c2_preview.launch \
    topomap_dir:=whill_odometry_ikuta_3.5 use_sim_time:=true compressed_input:=true

# 端末3（C2・自己位置推定に使うodom・座標変換のみを入力）
export ROS_MASTER_URI=http://localhost:11329
rosbag play --clock bag/1119/omnivla_edge_odometry_nearest_2026-11-19-07-42-51.bag \
    --topics /usb_cam/image_raw/compressed /odom /tf /tf_static
```

RVizでは Fixed Frame を `odom`、Path を `/placenav/gnm_path` にする。
このlaunchはCosPlaceとOmniVLA-edgeを使い、TEB、PD、深度ノード、車体ドライバーを起動しない。
実カメラのraw画像を使うときは `use_sim_time` と `compressed_input` を省略する。

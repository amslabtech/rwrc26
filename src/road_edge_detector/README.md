# road_edge_detector

## PlaceNav 統合版

Parrot の navigation.launch に `road_edge:=true` を追加して起動する。地図の対象区間を設定する前の bag 確認には `road_edge_target_gate:=false` を指定する。対象区間設定後は `road_edge_config:=/path/to/c2.yaml` で切り替えられる。

統合版は `/toponav/localization_diagnostics`（placenav_viz_msgs/LocalizationDiagnostics）を購読し、`msg.current_node_idx` を使う。以下は配布版の資料。整数トピックや整数を配信する評価ツールの記述は配布版に対応する。統合版のbagには親の診断トピックを記録する。

## 配布版 v0.3.2 の資料

C2 広角カメラの画像から、横断歩道へ進入する手前側の**道路端（歩道と車道の境界）**を検知する ROS 1（Noetic）パッケージです。
段差のある道路端だけでなく、色や舗装が変わるだけの段差の少ない境界、幅の広い段差、白っぽい帯も対象にします。

画像の上で道路端を検知して、その位置を配信します。
**実距離の推定と停止制御は行いません**（`/cmd_vel` や停止サービスは操作しません）。

- 走行中はずっと画像を処理し、ナビゲーションの**現在のノード番号**を使って、判定したい道路端（`config/c2.yaml` の `targets`。
  つくば本番コースの E01〜E14）の手前だけ検知結果を配信します。時間やフレーム番号は使いません。
- **現状（v0.3.2）**: tsukuba.bag（2026/10/17 の本番コースの走行、C2 の画像 43892 枚）の全フレームで、E01〜E14 の 14 か所すべてで
  正しい道路端だけを配信することを確認済み

参考実装: [amslabtech/rwrc23_stop_line_detector](https://github.com/amslabtech/rwrc23_stop_line_detector)
（画像の購読、OpenCV による処理、LSD による線分抽出、ROI の射影変換、連続検出、デバッグ画像、YAML によるパラメータ設定の考え方を流用し、
白線用の「2本の線に挟まれた白い長方形」の判定を、道路端用の判定に作り直しています）

---

## はじめに

### 何をするか

```
入力                                                       出力（ノード名 road_edge）
カメラ C2  /usb_cam/image_raw/compressed  ─┐                /road_edge/target          いま判定している道路端の名前（E01 など。判定区間外は空）
                                            ├→ road_edge →  /road_edge/detected        その道路端を検知しているか
ナビ  /toponav/localization_diagnostics/   ─┘   ノード      /road_edge/normalized_y    画像中央の列での道路端の位置（0〜1、大きいほど近い）
      current_node_idx（現在のノード番号）                  /road_edge/target_passed   その道路端を通過したと判断したか
                                                            /road_edge/debug_image/compressed  確認用の画像
```

### 動かすのに必要なもの（チェックリスト）

- [ ] **Ubuntu 20.04 + ROS Noetic**（ホストが別の版なら、同梱の `docker/` で Ubuntu 20.04 + ROS Noetic のコンテナを使える）
- [ ] **Python のライブラリ**: OpenCV・NumPy・PyYAML（apt）と、LSD の `ocrd-fork-pylsd==0.0.8`（pip）
- [ ] **カメラ**: C2 広角カメラ（2880×1860、約 120°）の圧縮画像（`sensor_msgs/CompressedImage`、JPEG、10 Hz）。
  **取り付け位置と向きは tsukuba.bag を取ったときと同じ**であること（ROI〈画像のどの範囲を見るか〉をこの取り付けに合わせてある）
- [ ] **ナビ**: 現在のノード番号を整数で配信していること（既定 `/toponav/localization_diagnostics/current_node_idx`）。
  `targets` のノード番号は地図 `whill_short_route_adaptive_c2_tsukuba_2.0` のもの。**地図が変わったら `targets` を直す**
- [ ] **CPU**: 1 フレームの処理（JPEG のデコード込み）が平均 100 ms 未満（10 Hz の画像に追いつくため。目安は 30〜70 ms）

### いちばん短い手順

```bash
# 1. 依存するものを入れる（初回だけ）
sudo apt install -y python3-opencv python3-numpy python3-yaml python3-pip python3-dev build-essential python3-catkin-tools
pip3 install ocrd-fork-pylsd==0.0.8
# 2. パッケージを catkin ワークスペースに置いてビルドする
cp -r road_edge_detector ~/catkin_ws/src/
cd ~/catkin_ws && catkin build road_edge_detector && source devel/setup.bash
# 3. 必要なものがそろっているかを確かめる（roscore 不要。すべて OK になればよい）
rosrun road_edge_detector check_setup.py
# 4. カメラとナビを起動した状態で起動する
roslaunch road_edge_detector road_edge_detector.launch
# 5. 出力を見る
rostopic echo /road_edge/target
rostopic echo /road_edge/normalized_y
```

---

## 1. ファイル一覧

### 動作に必要なのは、パッケージ `road_edge_detector/` だけ

```
road_edge_detector/
├── package.xml / CMakeLists.txt / setup.py   catkin パッケージの定義
├── README.md                                   この説明
├── config/c2.yaml                              パラメータ。★ 実機で変えることがあるのはこのファイルだけ
│                                               （判定したい道路端 targets、ナビのトピック名、ROI、しきい値）
├── launch/road_edge_detector.launch            起動用
├── scripts/road_edge_detector_node.py          ROS ノード（本体）
├── scripts/check_setup.py                      動かすのに必要なものがそろっているかの確認（roscore 不要）
├── scripts/evaluate_bag.py                     bag での評価（0913 の bag の場面台帳を使う）
└── src/road_edge_detector/                     処理の本体（ROS に依存しない Python のモジュール）
    ├── detector.py                             1 枚の画像での検知（RoadEdgeDetector）と連続検出（EdgeTracker）
    ├── target_gate.py                          判定したい道路端だけを出す絞り込み（TargetGate。ナビのノード番号を使う）
    ├── pipeline.py                             上の 3 つをまとめたクラス（RoadEdgePipeline。ROS ノードと評価ツールが使う）
    └── visualize.py                            デバッグ画像の描画
```

### bag で確認・調整するときに使うもの（任意）

開発に使ったフォルダ（`~/road_edge`）と同じディレクトリ構成にすると、この README のコマンドがそのまま使えます。

```
road_edge/
├── catkin_ws/src/road_edge_detector/   上のパッケージ
├── tools/
│   ├── run_full_bag.py                 bag の全フレーム（または一部の区間）で動かし、道路端ごとの確認用の画像と集計を作る
│   ├── node_track.py, pipeline_compat.py   run_full_bag.py が使う
│   ├── publish_node_from_track.py      ナビのノード番号が記録されていない bag を再生するときに、ノード番号の代わりを配信する
│   ├── tsukuba_node_track.csv          tsukuba.bag（10/17）の各時刻に当たる地図のノード（地図の画像との照合結果）
│   └── eval_tsukuba.py, tsukuba_gt.yaml, export_edge_frames.py   bag から書き出した画像での評価（「評価（rosbag）」の節）
├── docker/Dockerfile, docker/ros1.sh   Ubuntu 20.04 + ROS Noetic のコンテナ（ホストの OS の版に関係なく使える）
└── bags/                               記録した bag を置く場所
```

### 配布用ファイル

`~/road_edge/release/` に、上の構成をまとめた圧縮ファイルがあります（bag は入っていません）。

| ファイル | 中身 |
|---|---|
| `road_edge_detector_v0.3.2.tar.gz` | パッケージのみ |
| `road_edge_v0.3.2_bundle.tar.gz` | パッケージ + `tools/` + `docker/`（展開すると `road_edge/` ができる） |

自分で作るときは、パッケージだけなら次のとおりです（`__pycache__` は入れない）。

```bash
tar czf ~/road_edge_detector_v0.3.2.tar.gz --exclude='__pycache__' -C ~/road_edge/catkin_ws/src road_edge_detector
```

`package.xml` の `maintainer`（連絡先）と `license` は仮の値なので、渡す前に書き換えてください。

---

## 2. 必要な環境

| 項目 | 内容 |
|---|---|
| OS・ROS | Ubuntu 20.04 + ROS Noetic（`ros-noetic-ros-base` 以上。`rqt_image_view` で確認するなら `desktop`） |
| Python | Python 3.8（ROS Noetic の標準） |
| apt で入れるもの | `python3-opencv` `python3-numpy` `python3-yaml` `python3-pip` `python3-dev` `build-essential` `python3-catkin-tools`（`catkin build` を使うとき） |
| pip で入れるもの | `ocrd-fork-pylsd==0.0.8`（LSD による線分抽出。参考実装と同じ。rosdep では入らない。インストール時に C のコードをビルドするので `build-essential` と `python3-dev` が必要） |
| ROS のパッケージ | `rospy` `std_msgs` `sensor_msgs` `rostopic` `rosbag`（ROS Noetic に入っている） |
| カメラ | C2 広角カメラ。`sensor_msgs/CompressedImage`（JPEG）で 10 Hz。解像度は 2880×1860（ROI は画像の大きさに対する割合なので、解像度が違っても同じ範囲を見る） |
| ナビ | 現在のノード番号（整数の `data` を持つメッセージ。`std_msgs/Int32` など、型は自動で合わせる） |
| 計算時間 | 1 フレーム 30〜70 ms が目安（JPEG のデコード込み）。`check_setup.py` とノードのログで確かめられる |

`rosdep` を使うなら、apt で入れるもの（`package.xml` に書いたもの）は次で入ります。pylsd だけは pip で入れてください。

```bash
cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y
pip3 install ocrd-fork-pylsd==0.0.8
```

---

## 3. インストールとビルド

### ロボットの PC（ROS Noetic が入っている PC）

```bash
sudo apt install -y python3-opencv python3-numpy python3-yaml python3-pip python3-dev build-essential python3-catkin-tools
pip3 install ocrd-fork-pylsd==0.0.8

mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
tar xzf ~/road_edge_detector_v0.3.2.tar.gz          # → ~/catkin_ws/src/road_edge_detector
cd ~/catkin_ws
catkin build road_edge_detector                     # catkin_make のワークスペースなら catkin_make
source devel/setup.bash                             # 毎回の端末で（~/.bashrc に書いておくとよい）
rosrun road_edge_detector check_setup.py            # すべて OK になればよい
```

`check_setup.py` は、ライブラリ・設定ファイル（`targets` と道路端ごとの設定）を確かめ、合成した画像（歩道とアスファルトの境目を描いたもの）で
検知と処理時間を測ります。`--image 画像.jpg` を付けると、その画像で検知してデバッグ画像（`check_setup_debug.jpg`）を保存します。

### コンテナ（`docker/`）を使う場合

ホストの Ubuntu の版に関係なく、Ubuntu 20.04 + ROS Noetic と必要なものがすべて入った環境になります（Docker が必要）。

```bash
cd ~/road_edge/docker
./ros1.sh build          # 最初の 1 回だけ（イメージを作る）
./ros1.sh                # コンテナに入る（~/road_edge が /home/dev/road_edge に見える。もう 1 つ端末が要るときも同じコマンド）
# 以下コンテナ内
cd ~/road_edge/catkin_ws && catkin build road_edge_detector && source devel/setup.bash
```

---

## 4. 実行コマンド

### 起動

カメラ（`/usb_cam/image_raw/compressed`）とナビ（`/toponav/localization_diagnostics/current_node_idx`）が動いている状態で:

```bash
roslaunch road_edge_detector road_edge_detector.launch
```

| launch の引数 | 既定値 | 意味 |
|---|---|---|
| `image_topic` | `/usb_cam/image_raw/compressed` | 入力する圧縮画像のトピック |
| `config` | パッケージの `config/c2.yaml` | パラメータファイル（別のファイルを使うとき） |
| `num_detection` | 3 | 何フレーム続けて同じ位置に見えたら「検知」とするか |
| `visualize` | true | デバッグ画像を配信するか（購読者がいるときだけ作るので、true のままでも普段は重くならない） |
| `start_enabled` | true | 起動直後から検知するか（false なら `/road_edge/enable` に true が来るまで止めておく） |
| `use_target_gate` | true | ナビのノード番号で絞るか。false なら、見つけた道路端をすべて配信する（ナビの無い bag で試すとき用） |

例: `roslaunch road_edge_detector road_edge_detector.launch image_topic:=/camera/image_raw/compressed`

### 動いているかの確認

```bash
rostopic echo /road_edge/target                     # 道路端の判定区間に入ると E01 などが出る（判定区間外は空）
rostopic echo /road_edge/normalized_y               # 道路端を検知している間は 0〜1、それ以外は nan
rostopic hz /road_edge/detected                     # 画像と同じ 10 Hz 前後で出ていればよい
rqt_image_view /road_edge/debug_image/compressed    # マゼンタの太線 = 配信している道路端
```

起動したときのログに「判定する道路端」「道路端ごとの設定」が、道路端の判定区間に入った・出たとき、検知したとき・途切れたときにもログが出ます。
100 フレームごとに処理時間（平均・最大）が出るので、平均が 100 ms を超えていないかを見てください。

### 止める・再開する

```bash
rostopic pub -1 /road_edge/enable std_msgs/Bool "data: false"   # 止める（画像を処理せず、未検知を配信する）
rostopic pub -1 /road_edge/enable std_msgs/Bool "data: true"    # 再開
```

### ナビ無しで試す

ナビのノード番号が届かない間は、何も配信しません（「…がまだありません。届くまで道路端の検知結果は配信しません」とログが出る）。
カメラだけで試すときは、ノード番号で絞らずに起動します。

```bash
roslaunch road_edge_detector road_edge_detector.launch use_target_gate:=false
```

bag を再生して試す方法は「実行方法（rosbag で確認する場合）」の節にあります。

---

## 5. 入力と出力

ノード名 `road_edge`（launch の既定）で起動したときのトピック名です。

### 入力

| トピック | 型 | 内容 |
|---|---|---|
| `/usb_cam/image_raw/compressed`（launch の `image_topic`） | `sensor_msgs/CompressedImage` | C2 の画像（JPEG）。最新の 1 枚だけを処理する（処理が追いつかないときは古い画像を捨てる） |
| `/toponav/localization_diagnostics/current_node_idx`（`c2.yaml` の `node_topic`） | 整数の `data` を持つメッセージ（`std_msgs/Int32` など） | ナビの現在のノード番号。最後に届いてから `node_timeout`（3 秒）を過ぎると、検知結果を配信しない |
| `/road_edge/enable` | `std_msgs/Bool` | 任意。false で検知を止める、true で再開 |

### 出力（画像 1 枚ごとに配信）

| トピック | 型 | 内容 |
|---|---|---|
| `/road_edge/target` | `std_msgs/String` | いま判定している道路端の名前（`targets` の `name`、例 `E03`）。判定区間外では空文字 |
| `/road_edge/detected` | `std_msgs/Bool` | 判定している道路端を検知しているか |
| `/road_edge/normalized_y` | `std_msgs/Float32` | 道路端の位置。**画像中央の列（ロボットの正面）**での縦位置を画像の高さで割った値。0 = 画像の上端、1 = 下端で、**大きいほどロボットに近い**。検知していないときは NaN |
| `/road_edge/pixel_y` | `std_msgs/Float32` | 同じ位置を元画像のピクセルで（C2 なら 0〜1859）。検知していないときは NaN |
| `/road_edge/target_passed` | `std_msgs/Bool` | 判定している道路端の上を通り過ぎた（画像の下に出ていった）と判断したら true。その判定区間では以後配信しない |
| `/road_edge/detected_raw` | `std_msgs/Bool` | そのフレーム 1 枚だけで道路端らしい線を見つけたか（確認用。ノード番号でも絞らない） |
| `/road_edge/debug_image/compressed` | `sensor_msgs/CompressedImage` | デバッグ画像（購読者がいるときだけ。header は入力画像と同じ）。見方は「デバッグ画像の見方」の節 |

### 値の意味と注意

- 検知していないとき（判定区間外、判定区間に入ってすぐ、通過した後、ナビのノード番号が届かないとき）は `detected = false`、`normalized_y = NaN`
- 位置は、同じ位置の線が 3 フレーム続いたら「検知」とし、その後は追いかけて更新します（一瞬見えなくなっても 2 フレームまでは直前の位置を保つ）
- 道路端が斜めで画像中央の列まで届いていないときは、中央に近い方の端の位置を出します（E06 の見え始めなど）
- 段差の上面や側溝のように道路端が幅のある帯に見える場所では、帯の手前側の線を出すことも、向こう側の線を出すこともあります（画像の下の方で 0.05〜0.1 の違い）
- 出力のトピックには header がありません。すべて同じ画像の処理の中で続けて配信するので、それぞれの最新の値を組み合わせて使ってください
- 画像が届いてから配信するまでに、処理時間（30〜70 ms）ぶん遅れます

---

## 6. ロボットに組み込むときに確認すること

1. **カメラ**: トピック名（違えば `image_topic`）、圧縮画像（JPEG）で 10 Hz 前後か。**取り付け位置と向き**が tsukuba.bag のときと同じか
   （変えると、見る範囲〈ROI〉と、道路端が画像のどの高さに見えるかが変わる。ROI の決め方は「ROI（C2 用 v1）の決め方の記録」の節）
2. **ナビ**: ノード番号のトピック名（違えば `c2.yaml` の `node_topic`）。**地図が同じか**（`targets` のノード番号は
   `whill_short_route_adaptive_c2_tsukuba_2.0` のもの。地図を作り直したら、道路端ごとのノード番号を `targets` に書き直す）
3. **launch**: ロボットの launch から読み込む（下の例）
4. **計算時間**: ノードのログの処理時間が平均 100 ms 未満か。ほかの重い処理と CPU を取り合うと取りこぼす
5. **停止制御**: このノードは止めない。`/road_edge/target`・`/road_edge/detected`・`/road_edge/normalized_y`・
   `/road_edge/target_passed` を購読して、止めるかどうかは停止制御の側で決める（下の例）
6. **記録**: 走行するときは、画像・ナビのノード番号・このノードの出力を bag に記録しておく（「走行を記録して、後で確かめる」）

### ほかの launch に組み込む

```xml
<include file="$(find road_edge_detector)/launch/road_edge_detector.launch">
  <arg name="image_topic" value="/usb_cam/image_raw/compressed"/>
  <!-- 設定ファイルを別に用意したときだけ -->
  <!-- <arg name="config" value="$(find my_robot)/config/road_edge_c2.yaml"/> -->
</include>
```

---

## 7. 停止制御などからの使い方

- `target` が空でなく、`detected` が true の間は、`normalized_y` に判定している道路端の位置が入っている（大きいほど近い）
- 停止の例: `normalized_y` があるしきい値（ロボットを止めたい位置で、実際に測って決める）を超えたら止まる
- `target_passed` が true になったら、その道路端は通り過ぎた（渡り始めた）とみなす。次の道路端の判定区間に入るまで `target` は空か同じ名前のまま
- 道路端ごとに止まる位置を変えたいときは、`target` の名前で分ける

Python での受け取りの例:

```python
#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Bool, Float32, String


class RoadEdgeListener(object):
    def __init__(self):
        self.target, self.passed, self.y = "", False, float("nan")
        rospy.Subscriber("/road_edge/target", String, self.on_target)
        rospy.Subscriber("/road_edge/target_passed", Bool, self.on_passed)
        rospy.Subscriber("/road_edge/normalized_y", Float32, self.on_y)

    def on_target(self, msg):
        self.target = msg.data

    def on_passed(self, msg):
        self.passed = msg.data

    def on_y(self, msg):
        self.y = msg.data
        if self.target and not self.passed and not math.isnan(self.y):
            rospy.loginfo("%s の道路端: normalized_y=%.3f", self.target, self.y)


if __name__ == "__main__":
    rospy.init_node("road_edge_listener")
    RoadEdgeListener()
    rospy.spin()
```

---

## 8. ROS を使わずに Python から使う

検知・連続検出・絞り込みは `RoadEdgePipeline` にまとまっていて、ROS ノードと同じ処理を呼べます（ROS は不要）。

```python
import sys, yaml, cv2
sys.path.insert(0, "/home/dev/road_edge/catkin_ws/src/road_edge_detector/src")   # catkin のワークスペースを source していれば不要
from road_edge_detector.pipeline import RoadEdgePipeline
from road_edge_detector.detector import decode_compressed

cfg = yaml.safe_load(open("/home/dev/road_edge/catkin_ws/src/road_edge_detector/config/c2.yaml"))
pipe = RoadEdgePipeline(cfg)                          # use_target_gate=False ならノード番号で絞らない
img, orig_size = decode_compressed(jpeg_bytes, 720)   # または img = cv2.imread("frame.jpg"); orig_size = None
out = pipe.process(img, orig_size, stamp=t, node=current_node_idx)   # 画像 1 枚ごとに呼ぶ
if out.detected:
    print(out.target, out.normalized_y, out.pixel_y)
```

`out` には `detected`, `normalized_y`, `pixel_y`, `target`, `state`（`off` / `approach` / `passed`）, `detected_raw`, `near_ignored`,
`track_pts`（配信している線の点列）, `res`（検知器の途中結果）が入っています。デバッグ画像は
`road_edge_detector.visualize.draw_debug(img, out.res, confirmed=out.detected, track_y=out.normalized_y, track_pts=out.track_pts)` で作れます。

---

## 9. 判定したい道路端を変える・増やす（コースや地図が変わったとき）

`config/c2.yaml` の `targets` に、名前と、道路端のノード番号（地図のノード。道路端が画像の中ほどに見えるあたり）を書きます。

```yaml
targets:
  - {name: E01, node: 159}
  - {name: E15, node: 950, before: 2, after: 3}   # この道路端だけ判定区間をノード -2〜+3 にする
```

- 判定区間（`before` / `after`、既定 3 / 4）は、道路端が見え始めるノードから、止まって渡り終えるノードまでを含むようにします
- 判定区間の始まりを早くしすぎると、手前の歩道の中の線や、渡らない道路の道路端を拾いやすくなります（E06・E09・E12 は始まりを遅くしています）
- 地図を作り直してノード番号が変わったら、`targets` のノード番号も直します
- 変えたら、記録した bag で確かめます（「走行を記録して、後で確かめる」）

---

## 10. 道路端ごとの調整のしかた（E06 の例）

ふつうの設定で検知できない道路端は、`targets` のその項目に `detector:` / `tracker:` / `gate:` を書くと、**その道路端の判定区間の中だけ**
パラメータを変えられます。名前は `c2.yaml` の他の設定と同じです（ほかの道路端には影響しません）。

E06 は交差点の角で、道路端が右下がりに大きく斜めに見え（射影後の画像で 35〜50°）、すぐ向こうに車線の白線と横断歩道の白線があります。
ふつうの設定では、斜めの道路端が線の候補にならず、白線の方を選んでいました。次のように変えています。

```yaml
  - name: E06
    node: 315
    before: 1
    detector:
      max_angle: 65            # 射影後の画像で 35° より急な線分も使う
      min_coverage: 0.2        # 斜めの線は横方向の長さが短いので緩める（既定 0.35）
      step_th: 1.2             # 歩道とアスファルトの明るさの差が小さいので緩める（既定 2.0）
      pass_ratio: 0.5          # （既定 0.6）
      cw_enable: false         # 「横断歩道の白線のすぐ手前の線」を選ぶ処理を使わない
      paint_exclude: true      # 白い塗装（横断歩道・車線の白線）の縁は道路端にしない
      edge_angle_min: 5        # 元画像で右下がり 5〜40° の線だけ（道路端は +11〜+16°、車線の白線は右上がり）
      edge_angle_max: 40
    tracker:
      detection_count_th: 2    # 見える時間が短いので 2 フレームで確定（既定 3）
    gate:
      pass_y: 0.80             # 道路端は画像の左下へ出ていくので、0.80 より下まで来て見えなくなったら通過（既定 0.90）
      pub_min_y: 0.64          # 見え始めは ROI の上の縁にかかるので、そこから配信する（既定 0.655）
```

調整の手順:

1. `config/c2.yaml` の、その道路端の項目を書き換える
2. その道路端の前後だけ、bag で動かす（bag の索引で途中から読むので数分。0.1 秒おきの確認用の画像と、1 枚ずつのデバッグ画像を作る）
   ```bash
   cd ~/road_edge
   python3 tools/run_full_bag.py --bag bags/tsukuba/tsukuba.bag --node-track tools/tsukuba_node_track.csv \
       --pkg catkin_ws/src/road_edge_detector --out eval/tune_E06 --start 2372 --duration 85 \
       --only E06 --tile-interval 0.1 --max-tiles 60 --debug-frames
   ```
3. `eval/tune_E06/review/E06.jpg` でマゼンタ（配信した線）を見る。違うときは `eval/tune_E06/debug/` のデバッグ画像の右側
   （射影後の画像。水色 = 候補になった線、オレンジ = 条件を満たさなかった線、黄色 = 点字ブロックとして除いた線、数字 = 段差の強さ）で、
   正しい線が候補になっているかを見る。`summary.txt` で、その道路端を 1 回だけ配信して「通過と判断 はい」になっているかも見る
4. 決まったら、走行全体（`run_full_bag.py` を `--start` なしで）でも確認する

よく使う設定:

| 症状 | 変える設定（`detector:` / `tracker:` / `gate:`） |
|---|---|
| 正しい線が候補にならない（デバッグ画像の右側に線が無い） | 斜めなら `max_angle` を上げる、短いなら `min_coverage` を下げる |
| 正しい線がオレンジ（条件を満たさない） | `step_th`・`pass_ratio` を下げる（下げすぎると影や汚れも拾う） |
| 白線（横断歩道・車線）を選んでしまう | `paint_exclude: true`、横断歩道の手前の別の線を選ぶなら `cw_enable: false` |
| 向きの違う線（右上がりの白線など）を選んでしまう | `edge_angle_min` / `edge_angle_max`（元画像での傾き。右下がりが正） |
| 見え始めが遅い・見える時間が短い | `tracker:` の `detection_count_th` を 2 に、`gate:` の `pub_min_y` を下げる |
| 通り過ぎた後に別の線を出し続ける | `gate:` の `pass_y` を下げる（斜めの道路端は画像の横から出ていくので 0.90 まで下りてこない） |
| 近づく前に別の線を出す | 判定区間の始まりを遅くする（`before` を小さく）、`gate:` の `acq_y` を下げる |

---

## 11. 走行を記録して、後で確かめる

走行するときは、画像・ナビのノード番号・このノードの出力を記録してください。

```bash
rosbag record -O run_$(date +%m%d_%H%M).bag /usb_cam/image_raw/compressed \
    /toponav/localization_diagnostics/current_node_idx /toponav/localization_diagnostics/subgoal_idx \
    /road_edge/detected /road_edge/normalized_y /road_edge/pixel_y /road_edge/target /road_edge/target_passed
```

記録した bag は、`tools/run_full_bag.py` で走行全体を確かめられます（bag に記録されたナビのノード番号をそのまま使います）。

```bash
cd ~/road_edge
python3 tools/run_full_bag.py --bag bags/run_XXXX.bag --pkg catkin_ws/src/road_edge_detector --out eval/run_XXXX
```

| 出力 | 内容 |
|---|---|
| `eval/run_XXXX/review/Exx.jpg` | 道路端ごとの確認用の画像（判定区間に入ってから 0.5 秒おき）。マゼンタの太線が判定したい道路端に乗っているかを見る |
| `eval/run_XXXX/summary.txt` | 道路端ごとに、配信した回数・フレーム数・最初に配信した位置・通過と判断したか。判定区間外で配信した回数（0 のはず） |
| `eval/run_XXXX/events.csv` / `frames.csv` / `timeline.png` | 配信した区間の一覧 / フレームごとの結果 / 走行全体のグラフ |

tsukuba.bag のようにナビのノード番号が記録されていない bag では、`--node-track`（地図の画像との照合結果）で代わりにします
（「走行全体での確認（tools/run_full_bag.py、v0.3）」の節）。

---

## 12. うまく動かないとき

| 症状 | 確認すること |
|---|---|
| 起動するとエラーで止まる | `rosrun road_edge_detector check_setup.py` で NG の項目を直す（pylsd が無い、`source devel/setup.bash` をしていない、など） |
| 何も配信されない（`/road_edge/target` がずっと空） | ナビのノード番号が届いているか（`rostopic hz /toponav/localization_diagnostics/current_node_idx`）。3 秒（`node_timeout`）途切れると配信しない。ナビ無しで試すなら `use_target_gate:=false` |
| `target` は出るが `detected` が true にならない | デバッグ画像で赤（その 1 枚で選んだ線）が出ているか。赤が正しいのにマゼンタが無いなら、上部の状態表示の `(near line ignored)`（急に手前に現れた線）や `passed`（通過済み）を確認 |
| 画像が届いていない | `rostopic hz /usb_cam/image_raw/compressed`。トピック名が違えば launch の `image_topic` |
| 処理が追いつかない | ログの処理時間（100 フレームごと）。ほかの重い処理と CPU を取り合っていないか。デバッグ画像は購読者がいるときだけ作るので、`rqt_image_view` を閉じると軽くなる |
| ある道路端で誤る・検知しない | 「道路端ごとの調整のしかた」の手順で、その道路端だけ調整する |
| カメラの取り付けを変えた | 道路端の見え方が変わるので、ROI と各道路端の結果を bag で確かめ直す |

---

以下は、詳しい説明・パラメータの一覧・評価の記録です。

## 実行方法（rosbag で確認する場合）

### ナビのノード番号も記録した bag（次の走行から）

画像とナビのノード番号だけを再生します（このノードの出力も記録してある bag では、`/road_edge/...` を再生すると、動かしているノードの出力とまざるため）。

```bash
# 端末1
roscore
# 端末2: ノードを起動
roslaunch road_edge_detector road_edge_detector.launch
# 端末3: デバッグ画像を見る
rqt_image_view /road_edge/debug_image/compressed
# 端末4: bag を再生（画像とナビのノード番号だけ）
rosbag play run_XXXX.bag --topics /usb_cam/image_raw/compressed /toponav/localization_diagnostics/current_node_idx
```

### ナビのノード番号が無い bag（0913 の bag など）

ノード番号で絞らずに起動します（`use_target_gate:=false`）。見つけた道路端をすべて配信します。端末を 4 つ使います。

```bash
# 端末1
roscore

# 端末2: ノードを起動（ノード番号で絞らない）
roslaunch road_edge_detector road_edge_detector.launch use_target_gate:=false

# 端末3: デバッグ画像を見る（プルダウンで /road_edge/debug_image/compressed を選ぶ）
rqt_image_view

# 端末4: bag を再生
rosbag play ~/road_edge/bags/tsukuba_stop_line/whill_footcamera_c2_0913_1.bag
```

検知結果の値は次のように確認できます。

```bash
rostopic echo /road_edge/detected
rostopic echo /road_edge/normalized_y
```

端末2 には、検知が確定したとき・途切れたときのログと、100 フレームごとの処理時間が表示されます。
処理時間の平均が 100 ms（C2 の 10 Hz の 1 周期）を超える場合は、フレームの取りこぼしが起きます。

### tsukuba.bag で、判定したい道路端だけが出るかを確認する（v0.3）

tsukuba.bag にはナビのノード番号が無いので、もう 1 つの端末で代わりを配信します（`--clock` と `use_sim_time` が必要）。

```bash
# 端末1
roscore
# 端末2
rosparam set use_sim_time true
roslaunch road_edge_detector road_edge_detector.launch
# 端末3: ノード番号の代わり（地図の画像との照合結果から、再生時刻に当たるノードを配信する）
rosparam set use_sim_time true
python3 ~/road_edge/tools/publish_node_from_track.py _bag:=$HOME/road_edge/bags/tsukuba/tsukuba.bag \
    _track:=$HOME/road_edge/tools/tsukuba_node_track.csv
# 端末4: E03（ノード 209）の手前から再生（カメラの画像だけ）
rosbag play --clock ~/road_edge/bags/tsukuba/tsukuba.bag -s 1045 -u 40 --topics /usb_cam/image_raw/compressed
# 端末5: どの道路端を判定しているか
rostopic echo /road_edge/target
```

### launch の引数

| 引数 | 既定値 | 意味 |
|---|---|---|
| `image_topic` | `/usb_cam/image_raw/compressed` | 入力する圧縮画像のトピック |
| `config` | `config/c2.yaml` | パラメータファイル |
| `num_detection` | `3` | 何フレーム連続で検知したら「検知確定」とするか（参考実装の `num_detection` と同じ意味） |
| `visualize` | `true` | デバッグ画像を配信するか |
| `start_enabled` | `true` | （v0.2）起動直後から検知するか |
| `use_target_gate` | `true` | （v0.3）判定したい道路端のノードの手前だけ配信するか。ナビのノード番号が無い bag で試すときは `false` |

例: `roslaunch road_edge_detector road_edge_detector.launch num_detection:=1`

### ノードの出力を記録する

```bash
rosbag record -O road_edge_output.bag \
  /road_edge/detected /road_edge/detected_raw /road_edge/pixel_y /road_edge/normalized_y \
  /road_edge/target /road_edge/target_passed /road_edge/debug_image/compressed
```

デバッグ画像は 1 枚 100 KB 前後あるので、長い走行では外すか、走行の記録（「走行を記録して、後で確かめる」）だけにしてください。

---

## 出力トピック

ノード名 `road_edge`（launch の既定）で起動したときのトピック名です。すべて入力画像 1 枚ごとに 1 回配信します。

| トピック | 型 | 内容 |
|---|---|---|
| `/road_edge/detected` | `std_msgs/Bool` | 道路端の検知が確定しているか（位置のそろった検知が `num_detection` フレーム続いたら確定。確定後は `max_miss` フレームまでの途切れを許す） |
| `/road_edge/pixel_y` | `std_msgs/Float32` | 道路端の代表位置の縦座標 [px]（**元画像**、C2 なら 0〜1859）。確定中に検知が途切れたフレームでは直前の位置。確定していないときは NaN |
| `/road_edge/normalized_y` | `std_msgs/Float32` | `pixel_y / 元画像の高さ`（0〜1）。解像度が変わっても比較できる値。確定していないときは NaN |
| `/road_edge/debug_image/compressed` | `sensor_msgs/CompressedImage` | デバッグ画像（`visualize:=true` かつ購読者がいるとき）。**header は入力画像と同じ**（stamp・frame_id） |
| `/road_edge/detected_raw` | `std_msgs/Bool` | そのフレーム単体で検知したか（連続検出の条件をかける前。ノード番号では絞らない。確認用に追加） |
| `/road_edge/target` | `std_msgs/String` | （v0.3）いま判定している道路端の名前（`targets` の `name`、例 `E03`）。判定区間外では空文字 |
| `/road_edge/target_passed` | `std_msgs/Bool` | （v0.3）判定している道路端を通過したと判断したか（その後は判定区間を出るまで `detected = false`） |

入力（v0.2 で追加）

| トピック | 型 | 内容 |
|---|---|---|
| `/road_edge/enable` | `std_msgs/Bool` | `false` の間は検知を止め、`detected = false`・`pixel_y = NaN` を配信する（画像の処理もしない）。`true` で再開。起動直後の状態は `start_enabled`（既定 true） |
| `/toponav/localization_diagnostics/current_node_idx` | 整数の `data` を持つメッセージ（`std_msgs/Int32` など。型は自動で合わせる） | （v0.3）ナビの現在のノード番号。トピック名は `node_topic` で変えられる。届くまで、または `node_timeout` 秒途切れると、検知結果を配信しない |

`enable` は、トポロジカルマップで横断歩道に近づくノードの区間だけ検知を有効にする使い方を想定しています。
道路端の手前で止まった後や横断中は、道路を渡った先の道路端も同じように見えるため、検知器だけでは区別できません（「既知の制約」）。

```bash
rostopic pub -1 /road_edge/enable std_msgs/Bool "data: false"   # 止める
rostopic pub -1 /road_edge/enable std_msgs/Bool "data: true"    # 再開
```

### pixel_y（道路端の代表位置）の定義

**画像中央の列（ロボットの正面方向）で、道路端のロボット側の線が写っている縦位置**です。

- 段差や幅のある帯のように道路端が複数の線に見える場合は、一番ロボット側（画像の下側）の線を使います。参考実装の `corner_level`（検知した矩形の下端 = 手前側）と同じ考え方です
- **v0.2**: 横断歩道の白線が見えているときは、白線にいちばん近い境界（道路端の帯の車道側の線になることが多い）を使います。
  点状ブロックの列が道路端に沿って並ぶ場所では、ブロックの列ではなく、舗装が変わる境界を道路端とします。
  道路端の帯の手前側・向こう側のどちらの線を報告しても「正しい道路端」として評価しています
- 検知した線が画像中央の列まで届いていない場合は、中央に近い方の端の点の縦位置を使います
- 参考実装の `stop_area` は、射影変換後の縦座標を比例配分で求める近似で、実際の位置と最大で画像の高さの 2 割程度ずれます。このノードでは射影変換の逆変換で元画像の座標に正確に戻しています

### デバッグ画像の見方

- 上部: `CONFIRMED`（検知確定）/ `CONFIRMED (hold)`（確定中だがこのフレームは途切れ、直前の位置を保っている）/
  `CONFIRMED (weak)`（v0.2。このフレームの選択とは違うが、追っている位置の近くの弱い線で追跡を続けている）/
  `CONFIRMED (handover)`（v0.2.1。追っていた線が画像の下に出ていくので、同じ帯の向こう側の線に引き継いだ）/
  `crosswalk`（v0.2。横断歩道の白線を見つけている）/
  `raw (n)`（このフレームだけ検知、n は連続数）/ `none`（検知なし）、配信している normalized_y と pixel_y、処理時間
- 左: 入力画像（縮小）。緑の台形 = ROI、水色の点線 = 横断歩道のいちばん手前の白線の縁（v0.2。これより向こうの線は道路端にしない）
  - **赤線** = このフレーム 1 枚だけで道路端として選んだ線。**配信するとは限らない**（確定前・通過後・判定区間外・急に手前に現れた線など）
  - **マゼンタの太線**（v0.3.1）= 配信している道路端の線全体。赤と同じ線なら、赤の周りがマゼンタで縁取られて見える。
    赤とマゼンタが別の場所にあるときは、連続検出が追っている線（マゼンタ）と、そのフレームだけの選択（赤）が違う
  - マゼンタの短い横線（両端に縦線）= 配信している値 pixel_y（画像中央の列での位置）
  - マゼンタが無いフレームは、何も配信していない（`/road_edge/detected` = false）
- 右: ROI を射影変換した画像（参考実装のデバッグ画像に相当）。灰色 = LSD の線分、水色 = 条件を満たした境界の候補、オレンジ = 満たさなかった候補、
  黄色 = 点字ブロックの縁として除いた候補（v0.3.1）、赤 = 選ばれた道路端、数字 = 段差の強さ
- 上部の状態に `(near line ignored)` が付くフレームは、連続検出は線を追っているが、急に手前に現れた線なので配信していない（v0.3.1）

---

## 判定したい道路端だけを出す（v0.3）

画像だけでは、判定したい道路端と、判定しなくていい道路端（道路を渡った先の道路端など）を区別しきれません。
渡った先の道路端も本物の道路端で、近づいていくときの見え方はほとんど同じだからです。
そこで、ナビゲーションの**現在のノード番号**（地図 `whill_short_route_adaptive_c2_tsukuba_2.0` のノード）を使って、
判定したい道路端のノードの手前だけ検知結果を配信します。地図上の位置で決めるので、走行ごとに道路端の来るタイミングが変わっても影響を受けません。

1. 現在のノードが `[node - target_before, node + target_after]`（既定 -3〜+4。1 ノードはおよそ 2.5 m）に入ったら、その道路端の判定区間に入る。
   判定区間に入ったときに連続検出をやり直す（判定区間外で追っていた線を持ち込まない）
2. 判定区間の中では、連続検出で確定した位置を配信する（`/road_edge/target` に道路端の名前）
3. 確定していた道路端が `pass_y`（0.90）より下まで来た後に見えなくなる、または `pass_jump`（0.15）以上向こうの線に移ったら、
   「通過した」と判断し、判定区間を出るまで配信しない（`/road_edge/target_passed = true`）。道路を渡り始めた後の向こう側の道路端に釣られないため。
   ただし、`pass_from`（0.80）より向こうから追ってきた線で、現在のノードが `node - pass_arm_before`（v0.3.1 から node - 1）以降のときだけ（近づく前に歩道の中の帯が画像の下に出ていっても、通過にしない）
4. 一度通過した、または判定区間を出た道路端は、ノードが判定区間より十分手前に戻るまで、もう一度は出さない（ナビのノード番号が前後に揺れても大丈夫なように）
5. （v0.3.1）**遠くから追ってきた線だけ配信する**: 連続検出が線を追い始めた位置が `acq_y`（0.75）より手前（画像の下）なら、その線は配信しない。
   判定したい道路端は、近づくにつれて画像の上（ROI の上端 0.65 付近）から下りてくるので、いきなり手前に現れる線は歩道の中の線（tsukuba.bag の E04・E11・E12 で配信していた誤り）。
   ただし、判定区間の中で最後に配信した位置の近く（`-reacq_back`〜`+reacq_fwd`）で見つけ直した線は配信する（人に隠れて一度見失った道路端を、手前で見つけ直したとき）。
   このとき通過の判断にも使わない（v0.3.0 では、歩道の中の線が画像の下に出ていったのを「通過」と判断し、その後に見えた本物の道路端を配信しなかった。E12）
6. （v0.3.1）`pub_min_y`（0.655）より上（ROI の上の縁にかかる位置）は、まだ配信しない。線の向こう側が ROI の外になり、位置があいまいなため（E08 の見え始め）

| 名前 | 既定値 | 意味 |
|---|---|---|
| `use_target_gate` | true | ノード番号で絞るか。false なら v0.2 と同じく、すべての検知を配信する（launch の引数 `use_target_gate:=false` でも変えられる） |
| `node_topic` | `/toponav/localization_diagnostics/current_node_idx` | ナビの現在のノード番号のトピック |
| `node_timeout` | 3.0 | ノード番号がこの秒数届かなければ、検知結果を配信しない |
| `target_before` / `target_after` | 3 / 4 | 判定区間の広さ（ノード数）。道路端ごとに `targets` の中で `before` / `after` を書くと、その道路端だけ変えられる |
| `pass_y` / `pass_jump` / `pass_from` / `pass_arm_before` | 0.90 / 0.15 / 0.80 / 1 | 通過の判断（上の 3）。`pass_arm_before` は v0.3.1 で 2 → 1（ナビのノード番号が 1 つ先に進んでいると、E04 で歩道の中の線の通過を本物の通過と判断していた。本物の道路端の通過はどれも node 以降） |
| `acq_y` | 0.75 | （v0.3.1）追い始めた位置がこれより手前の線は配信しない（上の 5） |
| `reacq_back` / `reacq_fwd` | 0.03 / 0.08 | （v0.3.1）最後に配信した位置からこの範囲で見つけ直した線は、手前でも配信する |
| `pub_min_y` | 0.655 | （v0.3.1）これより上の位置は配信しない（上の 6） |
| `targets` の `detector:` / `tracker:` / `gate:` | （E06 だけ） | （v0.3.2）その道路端の判定区間の中だけ、検知（`max_angle` など）・連続検出（`detection_count_th` など）・この表の値（`pass_y` など。`pass_arm_before` は `arm_before` とも書ける）を変える。「道路端ごとの調整のしかた」の節 |
| `targets` | E01〜E14 | 判定したい道路端の名前とノード番号。E06（ノード 315）と E09（ノード 746）は、角で曲がり終えてから道路端が見えるので、判定区間の始まりを遅くしている（`before: 1` と `before: 0`）。E12（ノード 863）はメモのノードが道路端に近く、860〜861 では道路端がまだ見えず歩道の中の線だけが見えるので `before: 1`（v0.3.1） |

注意
- ナビのノード番号が届かない間は何も配信しません。ナビのトピックが入っていない bag を再生して試すときは、
  `use_target_gate:=false` で起動するか、tsukuba.bag なら `tools/publish_node_from_track.py` でノード番号の代わりを配信します（「実行方法」）
- `tools/tsukuba_node_track.csv` は、tsukuba.bag（10/17）の各フレームが地図のどのノードに当たるかを、地図の画像と bag の画像の照合で求めたものです。
  この bag にはナビのノード番号が記録されていないので、評価と再生試験のときだけ、ナビの代わりに使います
- 実際のナビのノード番号は、この照合と 1〜2 ノードずれるかもしれません。次に走らせるときは `/toponav/localization_diagnostics/current_node_idx` も
  bag に記録してください（評価をナビの実際の値でやり直せます）

---

## パラメータ（config/c2.yaml）

解像度に依存する値は、画像サイズに対する割合で指定しています。

### 連続検出・入出力

| 名前 | 既定値 | 意味 |
|---|---|---|
| `detection_count_th` | 3 | 何フレーム連続で検知したら確定とするか（launch の `num_detection` で上書き） |
| `max_jump` | 0.06 | 追っている位置からこれより大きく離れた検知は「外れ値」として扱う（normalized_y） |
| `max_miss` | 2 | 確定後、何フレームまでの途切れ・外れ値を許すか。その間は直前の位置を保つ |
| `track_gate` | 0.03 | （v0.2）確定後、予測位置からこの範囲（normalized_y）の検知を同じ道路端とみなす |
| `support_step` | 1.0 | （v0.2）確定後、予測位置の近くにこの段差の強さ以上の線があれば、その線で追跡を続ける（道路端が一瞬弱く写っても途切れない） |
| `far_switch` | 0.03 | （v0.2）確定した道路端より、これ以上向こう（画像の上）の線には乗り換えない。白線や向こう側の道路端は、近づいている道路端より必ず向こうにあるため。白線の手前の線として選ばれた検知は例外 |
| `exit_y` | 0.93 | （v0.2.1）追っている線の予測位置がこれより下（画像の下端寄り）まで来たら、同じ道路端の帯の向こう側の線への引き継ぎを許す |
| `exit_band` | 0.12 | （v0.2.1）引き継ぐ線は、追っている線からこの範囲（normalized_y）の向こうまで。これより遠い線（向こう側の道路端など）には引き継がない |
| `pending_gate` | 0.02 | （v0.2.1）確定前でも、追っている位置からこの範囲に弱い線があれば追跡を保つ（確定までの数は増やさない）。0 で無効 |
| `pending_step` | 2.0 | （v0.2.1）その弱い線に求める段差の強さ |
| `pending_max` | 5 | （v0.2.1）その弱い線だけで続けて追跡を保つのは何フレームまでか（その間に別の線ばかり選ばれるときに、そちらの確定を妨げないため） |
| `last_gate` | 0.02 | （v0.2.2）予測位置から外れた検知でも、直前の位置からこの範囲（normalized_y）なら同じ道路端とみなす。幅の広い段差のように近い 2 本の線を交互に選ぶときに見失わないため |
| `support_up` | 0.012 | （v0.3.1）弱い線で追跡を続けるとき、今の位置よりこれ以上向こう（画像の上）へは動かさない。地面の線は前に進むロボットから遠ざからないため（E02 で横断歩道の白線の縁に乗り移っていた） |
| `start_enabled` | true | （v0.2）起動直後から検知するか。false なら `/road_edge/enable` に true が来るまで止めておく |
| `visualize` | true | デバッグ画像を配信するか |
| `debug_jpeg_quality` | 80 | デバッグ画像の JPEG 品質 |
| `debug_width` | 720 | デバッグ画像の左側（入力画像）の幅 [px] |
| `decode_reduction` | 1 | JPEG のデコード倍率。1 = 元の解像度でデコードしてから縮小（v0.2 の既定。OpenCV の版によらず同じ結果になる）。2 / 4 / 8 / auto = 縮小しながらデコード（速いが、コンテナの OpenCV 4.2 では効かず、版によって結果が少し変わる） |
| `process_width` | 720 | この幅に縮小して処理する [px]（C2 の 2880 の 1/4） |

### ROI

| 名前 | 既定値 | 意味 |
|---|---|---|
| `roi_units` | normalized | `normalized`: 0〜1 の正規化座標 / `pixel`: 入力画像のピクセル座標 |
| `roi_upper_left` | [0.30, 0.65] | 台形の上辺の左 [x, y] |
| `roi_upper_right` | [0.70, 0.65] | 台形の上辺の右 |
| `roi_lower_left` | [0.00, 0.98] | 台形の下辺の左（参考実装では画像の最下行に固定だったものを指定可能にした） |
| `roi_lower_right` | [1.00, 0.98] | 台形の下辺の右 |

### 線分の抽出と連結

| 名前 | 既定値 | 意味 |
|---|---|---|
| `clahe_clip`, `clahe_grid` | 2.0, 4 | CLAHE（局所的なコントラスト強調）の強さとタイル数（参考実装と同じ） |
| `blur_ksize` | 5 | ぼかしのカーネルサイズ（参考実装と同じ） |
| `lsd_scale` | 0.6 | LSD 内部の縮小率（参考実装と同じ） |
| `seg_min_len` | 0.03 | 残す線分の最小長（射影後の幅に対する割合） |
| `max_angle` | 35.0 | 横向きとみなす傾きの上限 [deg] |
| `link_angle` | 12.0 | つなぐ線分同士の角度差の上限 [deg] |
| `link_dist` | 0.012 | つなぐときの縦ずれの上限（射影後の高さに対する割合）。参考実装の `close_line_th` に相当 |
| `link_gap` | 0.20 | つなぐときの横の隙間の上限（射影後の幅に対する割合） |
| `min_coverage` | 0.35 | 境界の候補に必要な横方向の長さ（射影後の幅に対する割合）。参考実装の `line_length_th` に相当 |
| `max_rms` | 0.015 | 曲線への当てはまり誤差の上限（射影後の高さに対する割合） |

### 境界の判定

| 名前 | 既定値 | 意味 |
|---|---|---|
| `band_gap` | 0.012 | 境界から比較する帯までの隙間（射影後の高さに対する割合） |
| `band_gap2` | 0.035 | 細い帯（白線など）を除外するための、離れた位置の隙間 |
| `band_width` | 0.040 | 比較する帯の厚さ |
| `n_bins` | 8 | 線を何区間に分けて比べるか |
| `sigma_L`, `sigma_ab`, `sigma_T` | 18.0, 8.0, 0.45 | 明るさ・色・模様の差の目安（この値の差で「1」と数える） |
| `ref_height` | 0.12 | 「今いる路面」とみなす ROI 下端の帯の高さ |
| `step_th` | 2.0 | 区間ごとの「段差の強さ」の閾値 |
| `own_th` | 0.0 | 区間ごとの「今いる路面との差（向こう側 − 手前側）」の閾値 |
| `pass_ratio` | 0.6 | 両方の条件を満たす区間の割合の下限 |
| `rel_th` | 0.3 | 画像内で一番強い境界に対する相対的な強さの下限 |
| `band_ext` | 0.035 | 選んだ線の手前この範囲（正規化 y）に有効な線があれば、そちらを道路端とする |

参考実装の `hsv_lo` / `hsv_hi`（白色の判定）は、白色画素率に依存しない判定に変えたため使っていません。

### 点字ブロック・白線の縁・道路端の傾き・曲がった道路端（v0.3.1 / v0.3.2）

| 名前 | 既定値 | 意味 |
|---|---|---|
| `paint_exclude` | false | （v0.3.2）白い塗装（横断歩道・車線の白線）の縁を、道路端・支えの線から除く。線の片側が `paint_L`（145）以上に明るく、反対側より `paint_dL`（50）以上明るければ塗装の縁。道路端の上面が明るい場所もあるので、既定では使わず、E06 だけで使っている |
| `edge_angle_min` / `edge_angle_max` | なし | （v0.3.2）道路端にする線の、元画像での傾きの範囲 [deg]。正 = 右へ行くほど手前（画像の下）に下がる線。交差点の角のように、道路端が決まった向きに斜めに見える場所で使う（E06: 5〜40） |
| `tactile_enable` | true | 線のすぐ向こう側か手前側が黄色の線（点字ブロックの列の縁）を、道路端・支えの線・横断歩道の白線の候補から除く |
| `tactile_db` | 15.0 | 黄色とみなす色の下限（Lab の b。128 を 0 とした値）。点字ブロックは日なたで約 +23、日陰で +15 前後、アスファルトや平板ブロックは -5〜+7 |
| `tactile_amin` | -8.0 | 黄色とみなす色の a の下限（128 を 0 とした値）。緑のフェンスや草（約 -30）を黄色と間違えないため（0913 の bag3） |
| `tactile_frac` | 0.5 | 線の片側の帯のうち、黄色の画素がこの割合以上なら点字ブロックの縁 |
| `extend_enable` | true | 表示用: 見つけた線の両端から、向きがなめらかにつながる線分（横向きでないものも含む）をたどって線を延ばして描く。配信する pixel_y は変えない |
| `extend_link` / `extend_turn` / `extend_max_angle` / `extend_steps` | 0.05 / 35 / 75 / 8 | 延ばすときの、端どうしの距離の上限（射影後の幅に対する割合）、1 回に曲がってよい角度、たどる線分の傾きの上限、片側でたどる線分の数の上限 |

### 横断歩道の白線（v0.2）

| 名前 | 既定値 | 意味 |
|---|---|---|
| `cw_enable` | true | 白線を手がかりに道路端を選ぶか（false で v0.1 と同じ選び方） |
| `cw_dL` | 30.0 | 白線の縁とみなす、線の両側の明るさ（Lab の L, 0〜255）の差 |
| `cw_Lmin` | 120.0 | 白線の明るさの下限 |
| `cw_dLsame` | 20.0 | 同じ横断歩道の白線どうしの明るさの差の上限。道路端の明るい帯を白線と間違えないため |
| `cw_hmax` | 0.12 | 白線 1 本の高さの上限（normalized_y） |
| `cw_gap_ratio` | 2.5 | 白線どうしの隙間の上限（白線の高さに対する倍率） |
| `cw_min_stripes` | 2 | 横断歩道とみなす白線の本数 |
| `cw_single` | true | 白線が 1 本しか見えていなくても、次の 3 つを満たせば横断歩道とみなす |
| `cw_single_L` | 145.0 | 　白線の明るさの下限 |
| `cw_single_dL` | 50.0 | 　白線と周りの路面の明るさの差の下限 |
| `cw_single_same` | 12.0 | 　白線の向こう側と手前側の路面の明るさの差の上限（同じアスファルトの上の白線） |
| `cw_margin` | 0.003 | 白線の手前の縁から、これ以上手前の線だけを道路端の候補にする |
| `cw_search` | 0.2 | 白線の手前の縁から道路端を探す範囲（normalized_y） |
| `cw_step_th` | 1.0 | 白線の手前で道路端に求める段差の強さの下限（通常の `step_th` 2.0 より緩い） |

---

## ROI（C2 用 v1）の決め方の記録

参考実装の ROI は 640×480 の foot camera（真下の地面 約30cm〜1m を見るカメラ）用でした。
解像度比（横 4.5 倍、縦 3.875 倍）で C2 に換算すると上辺が y≈115 px になり、空や建物をほぼ全部含むため、
「道路端が ROI から明らかに外れる場合」に当たると判断して、C2 の画像を見ながら手動で決めました。

1. 6 本の bag から、道路端までの距離が異なる代表画像（far / mid / near）を保存した
2. far 画像で、道路端の一番奥の線の高さをクリックで測った（正規化 y）:
   bag1 0.757 / bag2 0.780 / bag3 0.708 / bag4 0.678 / bag5 0.692 / bag6 0.735
3. 最も高い bag4（0.678）より約 0.03 上を上辺とした（y = 0.65）
4. 左右は、広角の歪みの影響を抑えるため中央 4 割（x = 0.30〜0.70）とした
5. 画像下端の中央にロボットの一部が写り込む（y ≥ 約 1850 px）ため、下辺を y = 0.98 とした
6. 道路端が画面内にある代表画像 23 枚すべてで、道路端が ROI に入っていることを確認した

2880×1860 でのピクセル座標は次のとおりです（`roi_units: pixel` で指定する場合）。

| 点 | 正規化 | ピクセル（2880×1860） |
|---|---|---|
| 上辺の左 | (0.30, 0.65) | (864, 1208) |
| 上辺の右 | (0.70, 0.65) | (2015, 1208) |
| 下辺の左 | (0.00, 0.98) | (0, 1822) |
| 下辺の右 | (1.00, 0.98) | (2879, 1822) |

---

## 検知の仕組み

1. **射影変換**: 画像を `process_width` に縮小し、ROI の台形を長方形に変換する（参考実装と同じ決め方）
2. **線分抽出**: 明るさ（Lab の L）に CLAHE とぼかしをかけ、LSD で線分を探す
3. **線分の連結**: ほぼ横向きの線分のうち、端が近く、向きがそろい、高さのずれが小さいものをつなぐ。
   参考実装の連結は、高さの違う線をつなぐと端点の最左・最右を結んだ実在しない斜めの線ができることがあったため、作り直した。
   広角の歪みで緩く曲がった線は 2 次曲線で表す
4. **候補の選別**: 横方向に十分長い線（`min_coverage` 以上）を境界の候補にする
5. **境界の判定**: 候補の上側と下側に帯を取り、明るさ・色・模様の差（段差の強さ）を区間ごとに測る。
   - 線のすぐ近くと少し離れた位置の両方で差があることを条件にし、細い白線（車線など）を除外する
   - 境界の向こう側の方が「今いる路面（ROI の下端）」と違うことを確かめる
   - 条件を満たす区間が `pass_ratio` 以上あれば、有効な境界とする
6. **横断歩道の白線（v0.2）**: 候補の線のすぐ向こう側・手前側の明るさから「明るい帯」（上の縁と下の縁の組）を探す。
   明るさのそろった帯が白線らしい高さと隙間で 2 本以上並んでいるか、両側が同じアスファルトのとても明るい帯が 1 本あれば、横断歩道とみなす。
   横断歩道があれば、いちばん手前の白線より手前（`cw_search` 以内）で、**白線にいちばん近い境界**を道路端とする（条件は `cw_step_th` に緩める）。
   道路端の見た目が弱くても、白線の縁や向こう側の道路端と間違えない。点状ブロックの列や影は道路端より手前にあるので選ばれない
7. **道路端の選択（横断歩道がないとき）**: 有効な境界のうち、十分に強いもの（一番強い境界の `rel_th` 倍以上）で**一番手前**のものを選ぶ。
   手前側 `band_ext` 以内にも有効な線があれば、そちら（道路端の帯のロボット側の線）に寄せる
8. **位置**: 選んだ線を元画像の座標に戻し、画像中央の列での縦位置を pixel_y とする
   - （v0.3.1）点字ブロック（黄色）の縁の線は、5〜7 の候補から除く
   - （v0.3.1）表示用に、線の両端からなめらかにつながる線分をたどって延ばした線を描く（曲がった道路端など。pixel_y は変えない）
9. **連続検出（v0.2 で改良）**: 位置のそろった（`max_jump` 以内の）検知が `detection_count_th` フレーム続いたら確定とする。確定後は
   - 道路端の動き（下向きの速さ）から次の位置を予測し、`track_gate` 以内の検知で更新する
   - そのフレームの選択が別の線でも、予測位置の近くに弱い線（段差の強さ `support_step` 以上）があれば追跡を続ける
   - `far_switch` より向こう（画像の上）の線には乗り換えない（白線・向こう側の道路端への乗り換えを防ぐ）。白線の手前の線として選ばれた検知は例外
   - 別の位置の検知が `detection_count_th` フレームそろったら乗り換える（支えの線がある間は、白線の手前の線として選ばれた検知のときだけ）
   - `max_miss` フレームまでの途切れは直前の位置を保つ
   - （v0.2.1）追っている線（帯の歩道側の線）が画像の下（`exit_y` 以降）まで来て ROI から出ていくときは、
     `exit_band` 以内の向こうの線の検知を、同じ帯の車道側の線として引き継ぐ。停止位置の直前で道路端を見失わないため
   - （v0.2.2）予測位置から外れた検知でも、直前の位置から `last_gate` 以内なら同じ線として更新する。弱い線で追跡を続けたフレームでは、下向きの速さを変えない
   - （v0.2.1）確定前でも、追っている位置の `pending_gate` 以内に段差の強さ `pending_step` 以上の線があれば追跡を保つ。
     遠くの道路端は 1 フレームおきにしか選ばれないことが多く、確定が遅れていたため
   - （v0.3.1）弱い線で追跡を続けるときは、今の位置より `support_up` 以上向こうへは動かさない

---

## 評価（rosbag）

`evaluate_bag.py` は、bag の画像を 1 フレームずつすべて処理し、場面台帳（`scenes.csv`）と突き合わせて評価します。
roscore は不要で、再生速度や処理落ちの影響を受けないため、何度実行しても同じ結果になります。

```bash
rosrun road_edge_detector evaluate_bag.py \
  --bags "~/road_edge/bags/tsukuba_stop_line/whill_footcamera_c2_0913_*.bag" \
  --scenes ~/road_edge/bags/tsukuba_stop_line/scenes.csv \
  --out ~/road_edge/eval/run1 --video
```

| オプション | 既定値 | 意味 |
|---|---|---|
| `--bags` | （必須） | 評価する bag。ワイルドカード可。ファイル名末尾の数字（`..._3.bag` → 3）を scenes.csv の `bag` 列と対応させる |
| `--scenes` | （必須） | 場面台帳 |
| `--out` | `eval_out` | 出力先 |
| `--topic` | `/usb_cam/image_raw/compressed` | 画像トピック |
| `--config` | `config/c2.yaml` | パラメータ |
| `--num-detection` | YAML の値 | 連続検出のフレーム数 |
| `--max-miss` | YAML の値 | 確定後に許す途切れのフレーム数 |
| `--tail-exclude` | 1.0 | target 区間の end_s 直前で、検知率の計算から除く秒数 |
| `--video` | なし | 全フレームのデバッグ動画を保存する |

### 場面台帳（scenes.csv）

```
bag,start_s,end_s,label,edge_type,view,place,note
1,0.08,16.98,target,flat_color,front,D館出入口,舗装の色が少し変わる（灰色）だけで段差なし
1,16.98,32.6,negative,,,D館出入口,車道のみ
```

- 時刻は bag の先頭からの経過秒
- `label`: `target`（道路端が見える区間）/ `negative`（道路端が無い区間）/ `ignore`（評価しない。遠くて判別困難など）
- `edge_type`: `flat_color`（段差なし、色が変わるだけ）/ `low_curb`（段差の低い道路端）/ `wide_step`（幅広な段付き）/ `white_line`（白っぽい帯）
- `view`: `front`（正面）/ `oblique`（斜め前）

### 評価指標

| 指標 | 定義 |
|---|---|
| 対象場面の検知率 | target 区間のフレームのうち、`/road_edge/detected` が true になったフレームの割合。`view`・`edge_type` ごとにも集計する。end_s の直前 `--tail-exclude` 秒は、広角の歪みで道路端が画像中央から先に ROI の下へ出ることが多いため、別に数える |
| 初回検知位置 | target 区間で最初に検知が確定したフレームの時刻・normalized_y・pixel_y。区間の開始からの遅れと、end_s までの残り時間も記録する |
| 位置の跳び | target 区間で、確定している位置が 0.1 以上変わった回数。検知率は位置の正しさを含まないので、別の境界（奥の道路端など）への乗り換えの目安として記録する |
| 非対象区間の誤検知 | negative 区間で検知が確定したフレーム数と、連続した誤検知を 1 回として数えた回数・時刻・画像 |

### 出力

| ファイル | 内容 |
|---|---|
| `summary.txt` | 上の指標のまとめ（端末にも表示） |
| `scenes_result.csv` | 場面台帳の区間ごとの結果 |
| `events.csv` | 初回検知と誤検知の一覧（時刻・位置・画像ファイル） |
| `frames_all.csv` | 全 bag のフレームごとの結果を 1 ファイルにまとめたもの（共有用） |
| `<bag名>/frames.csv` | フレームごとの結果（検知・位置・途切れ中か・処理時間） |
| `<bag名>/timeline.png` | 時間と検知位置のグラフ（背景 緑 = target、赤 = negative、灰 = ignore） |
| `<bag名>/images/` | 初回検知（`first_*.jpg`）と誤検知（`fp_*.jpg`）のデバッグ画像 |
| `<bag名>/debug.mp4` | 全フレームのデバッグ動画（`--video` のとき） |

### つくば本番コースの bag（tools/eval_tsukuba.py）

tsukuba.bag（10/17、晴れ）から書き出した道路端 14 か所の画像（`tools/export_edge_frames.py` の zip）で評価します。
正解は `tools/tsukuba_gt.yaml`（画像中央の列での道路端の帯 [車道側の線, 歩道側の線] の時刻変化。
v0.2.1 で、E09 の曲がり始め（3711.6 秒）以降が点状ブロックの列の縁をたどっていたのを、道路端（アスファルトとの境目）に直した）。

```bash
# コンテナ内、~/road_edge で
python3 tools/eval_tsukuba.py --frames frames/tsukuba_edges --gt tools/tsukuba_gt.yaml \
    --pkg catkin_ws/src/road_edge_detector --out eval/tsukuba_v022 --debug-images
```

- フレームの分類: 道路端が見えている（近づいている間 / 手前で停止中）、見えていない（近づく前・停止中に画像の下に出ている・横断中）、
  渡らない道路の道路端が正面にある（E06 の待機、E09）、評価しない（帯が ROI の上端・下端にかかる、見えなくなった直後 1 秒）
- 見えているフレームで確定した位置が帯 ±0.02（E06・E09 は ±0.03）に入れば「正しい」、外れれば「位置違い」
- 出力: `summary.txt`（道路端ごとの表）、`frames.csv`、`timeline_Exx.png`（縦 = normalized_y、緑 = 正解の帯、青 = 正しい確定、赤 = 位置違い・誤検知）、
  `wrong/`（位置違い・誤検知・見逃しのデバッグ画像。緑の短い線 = 正解の帯）
- （v0.3）`--node-track tools/tsukuba_node_track.csv` を付けると、実機と同じくノード番号で絞った結果を評価する。
  `--review` も付けると、道路端ごとの確認用の画像 `review/Exx.jpg`（判定区間に入ってから 0.5 秒おき。マゼンタ = 配信した線（v0.3.0 までは位置の印だけ））を作る

### 走行全体での確認（tools/run_full_bag.py、v0.3）

tsukuba.bag の全フレーム（約 4.4 万枚）で、ROS ノードと同じ処理（検知・連続検出・ノード番号での絞り込み）を動かし、
判定したい道路端だけが出ているかを確かめます。ノード番号は `tools/tsukuba_node_track.csv` で代わりにします。40〜60 分ほどかかります。

```bash
# コンテナ内、~/road_edge で
python3 tools/run_full_bag.py --bag bags/tsukuba/tsukuba.bag --node-track tools/tsukuba_node_track.csv \
    --pkg catkin_ws/src/road_edge_detector --out eval/full_v032 --dump eval/full_v032/det.pkl
```

v0.3.2 で追加したオプション:

| オプション | 意味 |
|---|---|
| `--start 秒 --duration 秒` | bag の先頭からの時刻で、その区間だけ処理する（bag の索引で途中へ飛ぶので、区間が短ければ 1〜2 分） |
| `--only E06` | 確認用の画像を作る道路端を絞る（カンマ区切り） |
| `--tile-interval 0.1 --max-tiles 60` | 確認用の画像の間隔と、道路端ごとの最大枚数（既定 0.5 秒・24 枚） |
| `--debug-frames` | 確認用の画像と同じフレームのデバッグ画像（右側に射影後の画像と候補の線）を `debug/` に 1 枚ずつ保存する |
| `--node-topic` | bag にナビのノード番号（既定 `/toponav/localization_diagnostics/current_node_idx`）が記録されていれば、`--node-track` の代わりにそれを使う |

`--dump`（v0.3.1）を付けると、フレームごとの検知器の結果（候補の線の位置・強さなど、20 MB 程度）を保存します。
連続検出や絞り込みの調整を、bag を読み直さずに試せます（書き出した画像は bag の画像を JPEG で保存し直したもので、
1 枚ごとの検知が約 1 割のフレームで bag と少し違うため、走行全体の確認はこのファイルで行うのが確実）。

| ファイル | 内容 |
|---|---|
| `summary.txt` | 道路端ごとに、判定区間に入ったノード、配信した回数・フレーム数、最初に配信した位置、通過と判断したか。判定区間外で配信した回数（0 のはず）と、絞らなかった場合の回数 |
| `events.csv` | 配信した区間の一覧（bag の時刻・ノード・道路端・位置）。最後の列は確認結果の記入用 |
| `review/Exx.jpg` | 道路端ごとの確認用の画像（判定区間に入ってから 0.5 秒おき）。マゼンタの太線（配信した線）が判定したい道路端に乗っているかを見る。赤 = そのフレーム 1 枚だけで選んだ線（配信するとは限らない） |
| `timeline.png` | 走行全体の配信した位置（青）と、絞らなかった場合の位置（灰） |
| `frames.csv` | フレームごとの結果（v0.3.1 から、最後の列 `near_ignored` = 急に手前に現れた線なので配信しなかったフレーム） |
| `det.pkl` | （`--dump` のとき）フレームごとの検知器の結果 |

---

## 開発時の確認結果（参考）

C2 の bag 6 本から場面台帳に沿って選んだ 52 枚（1440×930 に縮小した JPEG）で、試作段階に確認した結果です。
正解位置は画像中央の列で目視で読み取り、±0.03 以内を「正しい位置」としました。

| | 参考実装（白線検知） | このノード |
|---|---|---|
| 道路端が画面内にある 23 枚: 正しい位置で検知 | 0 | 19 |
| 　位置違い | 0 | 4 |
| 　見逃し | 23 | 0 |
| 道路端が無い 21 枚: 誤検知 | 0 | 6 |

- 位置違い 4 枚のうち 2 枚は、道路端の帯の奥側の線や手前の影を選んだもので、ずれは 0.03 をわずかに超える程度
- 誤検知の内訳: フェンスの根元 4、横断後に見える向こう側の道路端 1、路面の緑色の塗装の境目 1
- 処理時間: 2880×1860 の JPEG のデコードを含めて 1 フレーム約 40 ms

### rosbag の全フレームでの評価（C2 の bag 6 本、正面からの接近のみ）

連続検出 3 フレーム、end_s 直前 1 秒は別扱い。改良前は run1 の実測、改良後は run1 のフレームごとの検知結果に
新しい連続検出をかけて計算した値（検知器の出力は同じなので、`evaluate_bag.py` で取り直すと同じ値になるはず）。

| | 改良前（途切れ許容 0） | 改良後（途切れ許容 2） |
|---|---|---|
| 対象場面の検知率 | 72.0% (705/979) | 93.0% (910/979) |
| 　flat_color / white_line / wide_step | 73.0% / 87.9% / 65.1% | 94.1% / 92.2% / 91.5% |
| 非対象区間の誤検知の回数 | 24 回 | 6 回 |
| 非対象区間で確定していたフレーム | 25.7% (152/591) | 42.1% (249/591) |

- そのフレーム単体での検知は対象区間の 88.8% で起きており、改良前は一瞬の途切れや、手前の道路端と奥の道路端を交互に選ぶことで連続検出がやり直しになっていた
- 改良後は誤検知が細切れにならず 1 回につながるため回数は減るが、確定しているフレームは増える（向こう側の道路端など、本当に境界に見える物を安定して捉えるため）
- 初回検知はどの bag も記録開始から 0.2 秒後（3 フレーム確定の最短）。道路端が録画の最初から ROI 内にあるため、
  検知できる距離の上限は、検知器ではなく録画開始位置と ROI 上端（0.65）で決まっている

### つくば本番コースの bag（v0.1.1 → v0.2.0 → v0.2.2、道路端 14 か所）

近づいている間（10 Hz、863 フレーム）と、道路端の手前で停止して道路端が見えている間（2 Hz、648 フレーム）に分けた、
正しい道路端を確定していたフレームの割合（括弧内は位置違い）。どの版も、v0.2.1 で E09 を修正した正解で評価した値
（v0.2.0 の README の 76.8% は修正前の正解での値。v0.2.1 は 82.2%）。

| 道路端 | 種類 | 近づく間 v0.1.1 | v0.2.0 | v0.2.2 | 停止中 v0.1.1 → v0.2.0 → v0.2.2 |
|---|---|---|---|---|---|
| E01 | 道路端（茶色の線入り）+ 点字鋲 | 37.5% (62.5%) | 81.2% (18.8%) | 81.2% (18.8%) | 100 → 100 → 100% |
| E02 | 道路端 + 点字鋲 | 39.6% (31.2%) | 87.5% (12.5%) | 85.4% (14.6%) | - |
| E03 | 角の道路端（内側の帯あり） | 82.8% (8.6%) | 91.4% (1.7%) | **100.0%** (0.0%) | - |
| E04 | 角の道路端（内側の帯あり） | 79.4% (6.3%) | 95.2% (0.0%) | **100.0%** (0.0%) | 45.0 → 100 → 100% |
| E05 | 切り下げ（コンクリート） | 41.5% (44.6%) | 84.6% (6.2%) | **87.7%** (1.5%) | 96.1 → 100 → 100% |
| E06 | 旋回しながら横断歩道へ | 0.0% (82.4%) | 0.0% (29.4%) | 0.0% (29.4%) | - |
| E07 | 段差の小さい帯 → 平板ブロック | 52.4% (0.0%) | 47.6% (0.0%) | 52.4% (0.0%) | 3.8 → 3.8 → 3.8% |
| E08 | 道路端 → 平板ブロック | 98.6% (1.4%) | 94.4% (0.0%) | **98.6%** (1.4%) | 97.2 → 96.6 → 100% |
| E09 | 角を曲がりながら（人が多い） | 6.2% (52.5%) | 53.8% (6.2%) | 53.8% (6.2%) | - |
| E10 | 曲線の道路端（斜め） | 1.5% (85.3%) | 72.1% (11.8%) | 72.1% (11.8%) | 0.0 → 100 → 100% |
| E11 | 角の道路端（帰り） | 86.0% (1.8%) | 82.5% (0.0%) | **94.7%** (0.0%) | - |
| E12 | 角の道路端（帰り） | 83.6% (14.8%) | 77.0% (9.8%) | **100.0%** (0.0%) | - |
| E13 | 道路端 + 点状ブロック | 83.7% (6.1%) | 96.9% (3.1%) | 96.9% (3.1%) | - |
| E14 | 広場の段差 → 道路端 | 46.2% (21.5%) | 69.2% (3.1%) | **75.4%** (3.1%) | - |
| 合計 | | 56.1% (26.4%) | 78.0% (5.7%) | **82.6%** (4.8%) | 53.2 → 67.6 → 68.5%（位置違い 15.4 → 0.9 → 0.9%） |

- 停止位置の直前（近づく間のうち、道路端の帯の中心が 0.85 より下の 182 フレーム）: 64.3% → 77.5% → **89.6%**。
  v0.2.0 までは、帯の歩道側の線が ROI の下に出ていくところで見失い、その後に見えた帯の車道側の線を「別の線」として扱っていた（E03, E04, E11, E12）
- 道路端が見えていないフレームで確定していた割合: 30.8% → 18.8% → 18.7%（誤検知の回数 45 → 37 → 35 回）。多くは、横断を始めた後に見える向こう側の道路端（E03, E11, E12）と、
  近づく前に見える歩道の中の帯（E03, E04, E11, E12）。どちらも、判定したい道路端の 6〜16 秒前・後のもの
- 処理時間（検知のみ）: 平均 31 ms、95% 54 ms（v0.1.1 とほぼ同じ）
- 別の PC（Docker 内）で v0.2.1 を評価した結果も、全 4910 フレームで一致した

### ノード番号で絞った結果（v0.3、つくば本番コースの道路端 14 か所）

`eval_tsukuba.py --node-track` で、ナビのノード番号の代わり（`tools/tsukuba_node_track.csv`）を使って評価した値。検知器と連続検出は v0.2.2 と同じ。

| | 絞らない（v0.2.2） | ノード番号で絞る（v0.3） |
|---|---|---|
| 近づく間に正しい道路端を確定（括弧は位置違い） | 82.6% (4.8%) | 82.6% (4.6%) |
| 停止中（括弧は位置違い） | 68.5% (0.9%) | 68.5% (0.0%) |
| 道路端が見えていないのに配信 | 524 フレーム（35 回） | 32 フレーム（8 回） |
| 渡らない道路の道路端（E06 の待機、E09 の手前）で配信 | 52 フレーム | 0 フレーム |

- 残った 32 フレームのうち 28 フレームは、判定している道路端そのものが、正解の区間の少し前・後（見え始め、停止の後に動き出すところ）に配信されたもの。
  別の線だったのは E04 の判定区間に入った直後の 4 フレーム（歩道の中の線、0.4 秒）
- ナビのノード番号が 1〜2 ノードずれた場合も試した: 近づく間の正しい確定は 77.4〜82.6%、見えていないのに配信は 18〜82 フレーム。
  ノード番号が先に進む（実際より向こうのノードを出す）と、判定区間に入るのが早くなり、近づく前の歩道の中の帯を配信しやすい

### 走行全体の確認への対応（v0.3.1、つくば本番コースの道路端 14 か所）

`run_full_bag.py`（v0.3.0）の確認用画像で見つけた誤り（E02・E04・E08・E11・E12）を直した。
書き出した画像を時刻順に 1 本につなぎ、v0.3.0 の 1 枚ごとの選択が bag と違うフレームを bag の選択に置き換えて再生した値
（書き出した画像は JPEG を保存し直したもので、1 枚ごとの検知が約 1 割のフレームで bag と少し違うため）。

| | v0.3.0 | v0.3.1 |
|---|---|---|
| 近づく間に正しい道路端を確定（括弧は位置違い） | 75.9% (6.3%) | 81.5% (5.1%) |
| 停止位置の直前（帯の中心が 0.85 より下） | 80.8% | 87.4% |
| 道路端が見えていないのに配信 | 67 フレーム（14 回） | 28 フレーム（9 回） |

| 誤り | v0.3.0 | v0.3.1 |
|---|---|---|
| E02 938.2 / 938.8 / 940.6 秒（白線の縁） | 0.702 / 0.685 / 0.741 | 0.702 / 0.722 / 0.789（道路端） |
| E04 1124.1 秒・E11 4012.9 秒（手前の線） | 配信 | 配信しない（急に手前に現れた線） |
| E08 3345.3 秒（少し手前） | 0.653 | 配信しない（ROI の上の縁） |
| E12 4040.4〜4040.9 秒（遠くの別の線） | 配信 | 判定区間外（判定区間を 862 から） |
| E12 4046.4〜4048.2 秒（歩道の中の線） | 配信 → 「通過」と判断 | 配信しない |
| E12 4048.2 秒以降の本物の道路端 | 配信しない | 4048.4 秒から配信 |

- E06 は 2376.7〜2377.8 秒に 0.67〜0.68 の線を配信するようになった（点字ブロックの縁を除いたため）。正しいかは確認待ち
- ナビのノード番号が照合結果から ±1 ずれた場合も、近づく間の正しい確定は 81.3〜81.5% で変わらない（v0.3.0 は +1 で E04 を落として 68.6%）。
  ±2 ずれると 70.8〜74.0% に下がる（-2 では通過の判断が遅れて渡った先の道路端を配信、+2 では E04 の判定区間で歩道の中の線を通過と判断）
- 0913 の bag 6 本（`evaluate_bag.py` と同じ評価を再生）: 対象場面の検知率 94.2% → 93.9%（bag4 の 3 フレーム。枯れ葉・土の帯の一部を点字ブロックの色と判断）、
  確定位置の跳び 0 回、非対象区間で確定していたフレーム 256 → 250
- 正解の帯（`tools/tsukuba_gt.yaml`）は E01・E02・E06 の一部が合っていない

### E06 の調整（v0.3.2）

v0.3.1 の走行全体の確認で、E06 以外の 13 か所はすべて正しかった。E06（ノード 315）は、交差点の角で道路端が右下がりに
大きく斜めに見え（射影後の画像で 35〜50°、元画像で右下がり 11〜16°）、見えるのは 2376.0〜2377.7 秒の約 1.7 秒だけ。v0.3.1 は、
道路端が線の候補にならず（`max_angle` 35°・`min_coverage` 0.35 の条件を満たさない、日陰で明るさの差が小さい）、
道路端の向こうの車線の白線を配信していた。

E06 の判定区間の中だけ設定を変えた（「道路端ごとの調整のしかた」の節）。書き出した画像（10 Hz）での結果:

| 時刻 | 2376.0 | 2376.5 | 2377.1 | 2377.7 |
|---|---|---|---|---|
| v0.3.1 | 配信なし | 配信なし | 0.674（車線の白線） | 0.684（白線） |
| v0.3.2 | 0.648（道路端） | 0.703（道路端） | 0.784（道路端） | 0.838（道路端） |

- 2376.0〜2377.9 秒の間、道路端を配信し続け、道路端が画像の左下に出ていったところで「通過」と判断。
  その後の横断歩道の前での待機（約 60 秒）では何も配信しない
- E06 以外の 13 か所は、走行全体の確認で保存した v0.3.1 の検知結果（`det.pkl`、全 43892 フレーム）を v0.3.2 で再生し、
  配信した値がすべてのフレームで v0.3.1 と同じであることを確認した

### rosbag の全フレームでの評価（0913 の bag 6 本、v0.1.1 → v0.2.1 → v0.2.2）

`evaluate_bag.py` で、場面台帳のとおりに評価した値（v0.1.1 = run2、v0.2.1 = run3）。
1 枚の画像での検知は 3 つの版で全 1682 フレーム同じで、違いはすべて連続検出による。

| | v0.1.1 | v0.2.1 | v0.2.2 |
|---|---|---|---|
| 対象場面の検知率 | 93.0% (910/979) | 91.7% (898/979) | **94.2%** (922/979) |
| 　flat_color / white_line / wide_step | 94.1 / 92.2 / 91.5% | 93.1 / 92.2 / 89.4% | 94.6 / 92.2 / 94.1% |
| 　bag 1 / 2 / 3 / 4 / 5 / 6 | 98.1 / 98.3 / 89.4 / 89.3 / 92.2 / 95.7% | 98.8 / 93.2 / 89.4 / 86.2 / 92.2 / 95.7% | 98.8 / 97.4 / 90.6 / 93.3 / 92.2 / 95.7% |
| 確定位置の跳び（0.1 以上） | 2 回 | 0 回 | 0 回 |
| 非対象区間で確定していたフレーム | 42.1% (249/591)、6 回 | 43.3% (256/591)、5 回 | 43.3% (256/591)、5 回 |

- v0.2.1 で下がったのは、幅の広い段差（bag4）などで検知器が近い 2 本の線を交互に選ぶ場面。予測位置からのずれが `track_gate` を超えて
  数フレーム見失っていた。v0.2.2 の `last_gate` で解消した
- bag3 の 11〜22 秒は、v0.2 以降の連続検出が、柵の基礎（段差の上面）の線を追い続ける。1 枚ごとの検知は手前の側溝との境目を選ぶことが多く、
  v0.1.1 はそちらを配信していた。道路端の帯の中の線なので位置違いにはしていないが、配信する位置は手前の線より 0.05〜0.09 向こうになる

---

## 既知の制約と今後

- **見た目が本当に「境界」である物は誤検知しうる**: フェンスや建物の根元、横断後に見える向こう側の道路端、路面の塗装の境目など。
  特に、道路端の手前で止まった後は向こう側の道路端が判定したい道路端と同じように見え、検知器だけでは区別できない。
  v0.3 では、ナビのノード番号で判定したい道路端の手前だけ配信し、通過した後は配信しないことで防いでいる。
  ただし、判定区間の中に見える別の線（歩道の中の帯など）は防げないので、ナビのノード番号の精度に左右される
- **つくばで残っている弱点（v0.2.2）**: E06（旋回中に約 1.5 秒しか見えない）、E07（段差の小さい帯。0.89 より下に来ると線が見つからず、停止中もほとんど検知できない）、
  E09 の近づき始め（人で道路端が隠れる）、E10（道路端の上に人が立っている・道路端が曲がっている）、E14 の終盤（道路端が 0.94 より下）
- **斜め・曲がった道路端**: ふつうの設定では、線分は射影後の画像で横から ±35° 以内のものだけを使い、ROI の幅の 35% 以上つながった線を候補にして、
  画像中央の列での位置を出す。E09（3712.7 秒）程度の斜めは検知できる。E06 のように大きく斜めの場所は、その道路端だけ設定を変えて対応した（v0.3.2）。
  交差点の角で手前に曲がってくる部分（E10 の左側）は線の候補にならない。斜めの道路端が画像中央の列まで届いていないときは、
  中央に近い方の端の位置を出す（E06 の見え始めと終わり）
- **ROI の上の縁と「急に手前に現れた線」の規則（v0.3.1）**: 道路端が人に隠れたまま手前（0.75 より下）まで来ると、見えた時点で配信しない
  （最後に配信した位置の近くで見つけ直した場合は配信する）。0.655 より上は配信しないので、配信の始まりがわずかに遅くなる
- **道路端の帯のどの線を配信するかは場面による**: 段差の上面や側溝のような幅のある帯では、帯の歩道側の線（手前）を配信することも、車道側の線（向こう）を配信することもある
  （v0.2.1 以降、帯の歩道側の線が画像の下に出ていくときは車道側の線に引き継ぐ。0913 bag3 では、柵の基礎の線を追い続ける）。
  停止の判断では、帯の幅（画像の下の方で normalized_y 0.05〜0.1）の違いがありうることを見込む
- **道路端が ROI の下端に近いと測れない**: 境界の手前側の帯が ROI からはみ出すため。停止の判断には、それより前の検知を使う想定。
  道路端が画像中央で見えなくなると、次に見える境界（向こう側の道路端など）に乗り換えることがある（評価の「跳び」で記録）
- **段差のある道路端では奥側の線を選ぶことがある**: 段差の側面とアスファルトの境目が影で見えにくいとき、段差の上面と土の境目を道路端として報告する
- **斜め前からの接近は未評価**: 0913 の bag は正面からの接近のみ（bag3 の序盤は約 25° 傾いて写るが検知できている）
- **C2 の camera_info は未設定**（K, D がすべて 0）: 歪み補正はしていない。CameraInfo と取付 TF から距離を計算する場合は、C2 のキャリブレーション（120° の広角なので魚眼モデルの可能性あり）が必要
- **出力の Float32 / Bool には header が無い**: 停止制御で距離推定や時刻同期が必要になれば、header 付きのメッセージの追加を検討する

---

## 版の履歴

**v0.2（つくば本番コースの bag で調整）**: 横断歩道の白線を手がかりに道路端を選ぶ処理と、連続検出の改良を加えました。
**v0.2.1**: 連続検出をさらに改良しました（道路端の帯の歩道側の線が画像の下に出ていくときに車道側の線へ引き継ぐ /
確定前でも弱い線で追跡を保つ）。
**v0.2.2**: 0913 の bag 全体での再評価で、幅の広い段差のように近い 2 本の線を交互に選ぶ場面で見失うことが分かったので、
直前の位置に近い検知も同じ線として扱うようにしました。検知器（1 枚の画像での判定）は v0.2.0 から変えていません。
つくば本番コースで判定したい道路端 14 か所で、近づいている間に正しい道路端を確定していたフレームは
v0.1.1 56.1% → v0.2.0 78.0% → v0.2.2 82.6%、位置違いは 26.4% → 5.7% → 4.8%、
停止位置の直前（道路端が画像の 0.85 より下）に限ると 64.3% → 77.5% → 89.6% です（詳しくは「評価（つくば本番コースの bag）」）。
0913 の bag 6 本の対象場面の検知率は 93.0%（v0.1.1）→ 94.2%（v0.2.2）、確定位置の跳びは 2 → 0 回です。

**v0.3（判定したい道路端だけを出す）**: 走行中はずっと検知を動かしたまま、ナビゲーションの現在のノード番号
（`/toponav/localization_diagnostics/current_node_idx`）を使い、判定したい道路端（`config/c2.yaml` の `targets`。E01〜E14）の
ノードの手前だけ検知結果を配信します。時間やフレーム番号は使いません。道路を渡り始めた後に見える向こう側の道路端は、
「判定している道路端を通過した」と判断してから配信しません（「判定したい道路端だけを出す」の節）。

**v0.3.1（走行全体の確認結果を反映）**: tsukuba.bag 全体で確認した結果（E02・E04・E08・E11・E12 の誤り）をもとに直しました。
- 近づく前に急に手前に現れた線（歩道の中の線）を配信しない（判定したい道路端は画像の上から下りてくるため）
- 弱い線で追跡を続けるときに、今の位置より向こうへは動かさない（横断歩道の白線の縁に乗り移っていた）
- 点字ブロック（黄色）の縁を道路端にしない（道路端は舗装の変わり目）
- ROI の上の縁にかかる位置（0.655 より上）はまだ配信しない
- デバッグ画像・確認用の画像で、配信している線全体をマゼンタで描く（v0.3.0 までは画像中央の位置の短い印だけだった）

**v0.3.2（E06 の調整・使い方の整理）**: v0.3.1 の走行全体の確認で、E06 以外の 13 か所はすべて正しいことを確認しました。
- 道路端ごとに、検知・連続検出・絞り込みのパラメータを変えられるようにした（`config/c2.yaml` の `targets` に `detector:` / `tracker:` / `gate:`）。
  E06（交差点の角の、右下がりに大きく斜めの道路端。見えるのは約 1.7 秒）をこれで調整した（「道路端ごとの調整のしかた」の節）
- 検知器に、白い塗装の縁を除く設定（`paint_exclude`）と、道路端の傾きの範囲の設定（`edge_angle_min` / `edge_angle_max`）を追加（どちらも既定は使わない）
- 検知・連続検出・絞り込みを `RoadEdgePipeline`（`src/road_edge_detector/pipeline.py`）にまとめ、ROS ノードと評価ツールが同じ処理を使うようにした。
  ROS を使わずに Python から呼ぶこともできる
- E06 以外の 13 か所の結果は v0.3.1 と同じ（走行全体の確認で保存した検知結果で全 43892 フレームを再生して確認）
- 使い方・組み込み方を README の前半にまとめた

**v0.3.2 の引き継ぎ用の整理**: この README の前半（「はじめに」〜「うまく動かないとき」）に、ファイル一覧・必要な環境・インストール・実行コマンド・
入力と出力・組み込み方をまとめた。必要なものがそろっているかを確かめる `scripts/check_setup.py` を追加し、`package.xml` に `rostopic` を足した
（検知の処理と結果は v0.3.2 のまま）。

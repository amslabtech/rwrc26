# Jetson AGX Xavier での PlaceNav 推論

GPU を使う `navigate.py`（VPR とウェイポイント推論）を、parrot につながった Jetson AGX Xavier で動かす。
Xavier はインターネットに接続しない。必要なものはすべて parrot（インターネット接続あり）で用意し、
parrot–Xavier 間の有線 LAN（`192.168.1.146` ↔ `192.168.1.246`）で渡す。

**Xavier の OS（JetPack 5.0.2 / L4T R35.1）と既存の Docker には手を付けない。**
eMMC には既存の環境があり空きも 3 GB しかないため、PlaceNav 用の Docker デーモンを別に立て、
イメージとデータは追加した NVMe SSD（`/mnt/ssd`）に置く。

```
parrot (x86, Internet)                       Xavier (JetPack 5.0.2 のまま, offline)
 build_image.sh  arm64 イメージ作成 ─┐        docker.service（既存, eMMC）… 変更しない
 setup_xavier.sh runtime deb 取得 ───┼─ LAN ─▶ setup_xavier_host.sh: fstab に SSD /
                                     │          nvidia-container-toolkit / docker-placenav.service
 deploy.sh       イメージ・src 配布 ─┘─ LAN ─▶ /mnt/ssd/PlaceNav (src, 重み, topomap)
                                               run_xavier.sh → check / bench / navigate_xavier.sh
```

| ファイル | 実行する場所 | 内容 |
|---|---|---|
| `build_image.sh` | parrot | `Dockerfile` から arm64 イメージを qemu でビルドし、`~/xavier_jp512/placenav-xavier.tar.gz` に保存 |
| `setup_xavier.sh` | parrot | NVIDIA コンテナ runtime の deb を取得し、Xavier で `setup_xavier_host.sh` を sudo 実行する（初回のみ） |
| `deploy.sh` | parrot | イメージ（未読込時のみ）と `src/`・`docker/`（重み・topomap を含む）を Xavier に送る |
| `run_xavier.sh` | Xavier | 推論コンテナで任意のコマンドを実行（parrot の ROS master に接続） |
| `navigate_xavier.sh` | Xavier | 走行時に `navigate.py` を起動（下記「実機での走行」） |
| `check_xavier.py` | Xavier（コンテナ内） | CUDA・cuDNN・cv_bridge の確認（qemu では確認できない分） |
| `common.sh` | ― | IP・ユーザ名・パスなどの設定。環境変数で上書き可 |

| 項目 | 内容 |
|---|---|
| 推論イメージ | `dustynv/l4t-pytorch:r35.4.1`（digest 固定）: Ubuntu 20.04, CUDA 11.4, **PyTorch 2.0.0**, torchvision 0.15.1（sm_72）+ ROS Noetic（rospy/msgs/cv_bridge のみ） |
| Xavier に追加するもの | `/etc/fstab` に SSD 1 行、NVIDIA コンテナ runtime の deb 4 つ（R35.1 用、`nvidia-docker2` は入れない）、`/etc/docker/placenav-daemon.json`、`docker-placenav.service` |
| 変更しないもの | 既存の `docker.service`、`/etc/docker/daemon.json`（存在しない）、eMMC 上のイメージ・コンテナ |
| PlaceNav 用 Docker | ソケット `unix:///run/docker-placenav.sock`、データ `/mnt/ssd/docker-placenav`、containerd 名前空間 `placenav`、既定 runtime nvidia。手で使うときは `docker -H unix:///run/docker-placenav.sock ...`（既定の `docker` は既存のデーモン） |

## 導入（parrot、このディレクトリで実行）

```bash
./build_image.sh    # 数十分（コンパイルなし）。Dockerfile を変えたときだけ再実行
./setup_xavier.sh   # 初回のみ。Xavier の sudo パスワードを聞かれる
./deploy.sh         # コード・重み・topomap を変えたら毎回（イメージを入れ直すときは --image）
```

- `setup_xavier.sh` の最後に、PlaceNav 用 Docker の情報（`root=/mnt/ssd/docker-placenav`, `default=nvidia`）と、
  既存 Docker が変わっていないこと（`active`, root `/var/lib/docker`）が表示される。
- `deploy.sh` は parrot 上の `src/model_weights` と `src/topomaps` をそのまま送る。
  **先に parrot 側で使うモデルの準備（`prepare_waypoint.py`, `prepare_vpr.py`, 重みのダウンロード）を済ませておくこと。**
  `global-feats-*.h5` は送らず、Xavier 上で初回起動時に作り直す。
- `deploy.sh` は上書きのみで、parrot 側で消したファイルは Xavier に残る。

## 動作確認と推論時間の計測（Xavier）

```bash
ssh amsl@192.168.1.246
sudo jetson_clocks   # 計測・走行前に毎回（再起動で戻る）。電力モードは sudo nvpmodel -q で確認（MAXN）
/mnt/ssd/PlaceNav/docker/xavier/run_xavier.sh python3 ../docker/xavier/check_xavier.py
/mnt/ssd/PlaceNav/docker/xavier/run_xavier.sh python3 bench_inference.py \
    --pr-models cosplace salad megaloc --wp-models omnivla_edge nomad gnm_large
```

5 Hz で回すには VPR + ウェイポイント推論の合計が 200 ms を十分下回る必要がある。
2026-11-14 の計測（中央値、ms）:

| VPR | ms | ウェイポイント | ms |
|---|---|---|---|
| CosPlace | 21.5 | GNM large | 20.9 |
| SALAD | 121.8 | OmniVLA-edge | 100.5 |
| MegaLoc | 121.1 | NoMaD | 186.0 |

使う組み合わせは **CosPlace + OmniVLA-edge**（約 122 ms）。

## LAN

parrot–Xavier 間は 1 Gbps であること（Xavier で `cat /sys/class/net/eth0/speed` が 1000）。
1280x720 の生画像は 1 Gbps なら約 30 ms で届くが、100 Mbps だと 1 枚 235 ms かかり欠落する
（2026-11-14 にハブ–Xavier 間の不良ケーブルで 100 Mbps になっていた）。

## 実機での走行（parrot で launch、Xavier で navigate.py）

```bash
# 1. parrot: いつもの launch に remote_navigation:=true を付ける（path_timeout > 0 でないと launch が拒否する）
roslaunch rwrc26_bringup navigation.launch remote_navigation:=true path_timeout:=1.0 \
    wp_model:=omnivla_edge pr_model:=cosplace topomap_dir:=<route> drive_robot:=false
# 2. Xavier（別ターミナル）: parrot の /placenav_runner のパラメータで navigate.py を起動する
sudo jetson_clocks
/mnt/ssd/PlaceNav/docker/xavier/navigate_xavier.sh
```

- `remote_navigation:=false`（既定）なら従来どおり parrot で `navigate.py` を動かす。
- route・モデル・各種インデックスは parrot の launch 引数だけで決まる。Xavier 側で指定するものはない。
  `deploy.sh` で重みを送ってあれば NoMaD なども選べるが、CosPlace + NoMaD は 1 周期が 200 ms を超えるので約 4 Hz になる。
- 起動順はどちらが先でもよい（Xavier 側は parrot の launch を最大 120 秒待つ）。
- `navigate_xavier.sh` は最初に chrony を確認する。時刻源が parrot でない、またはずれが 5 ms を超える
  （再起動直後に起きる）場合は起動しない。1 分ほど待って再実行する。
- parrot の launch を止める（または再起動する）と、Xavier 側の navigate.py も止まる。
- Xavier か LAN が止まると、parrot の TEB は `path_timeout` 秒で経路を失って停止する
  （`Bridge stop: INPUT_TIMEOUT`）。parrot の launch はそのまま残るので、Xavier で
  `navigate_xavier.sh` をもう一度実行すれば走行に戻る（位置は今の画像から地図全体で推定し直す）。
- `remote_navigation:=true` では parrot 側の PlaceNav GUI は出ない（位置推定の DB が Xavier 側にあるため）。rviz は出る。
- 初回は Xavier で `global-feats-<pr_model>.h5` を作るので起動に時間がかかる。
- 最初は `drive_robot:=false` で、Xavier のログの `Runtime` と rviz の経路を確認してから `true` にする。
  2026-11-15 の実機（CosPlace + OmniVLA-edge）: 1 周期 約 155 ms（前処理込み）、`/waypoint` 5.3 Hz。
  `Average runtime` は起動直後の遅い周期を含む累積平均なので、しばらく高めに出る。

## 元に戻す（Xavier）

```bash
sudo systemctl disable --now docker-placenav
sudo rm /etc/systemd/system/docker-placenav.service /etc/docker/placenav-daemon.json
sudo rm -rf /mnt/ssd/docker-placenav /mnt/ssd/PlaceNav
# 必要なら: /etc/fstab の SSD の行を削除（バックアップ /etc/fstab.before-placenav.* あり）
# 必要なら: sudo dpkg -r nvidia-container-toolkit libnvidia-container-tools libnvidia-container1 libnvidia-container0
```

## 注意点

- **R35.4.1 用のイメージを R35.1 のホストで動かしている**（CUDA 11.4 は共通、ドライバはホストのものを使用）。
  イメージを作り直したら Xavier で `check_xavier.py` を実行して CUDA・cuDNN・cv_bridge を確認する。
- イメージの PyTorch は 2.0.0（x86 イメージは 2.1.2）。x86 で書き出した VPR の TorchScript（`model_weights/vpr/*.pt`）を
  2.0 で読めない場合は、Xavier 上で `prepare_vpr.py --model <名前>` を再実行して書き出し直す
  （上流ソースと重みのキャッシュも Xavier に必要）。

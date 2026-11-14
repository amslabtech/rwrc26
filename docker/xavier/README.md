# Jetson AGX Xavier への PlaceNav 推論環境の導入

Xavier はインターネットに接続しない前提。必要なものはすべて parrot（インターネット接続あり）で用意し、
parrot–Xavier 間の有線 LAN（`192.168.1.146` ↔ `192.168.1.246`）で渡す。

**現在の方針: Xavier の OS（JetPack 5.0.2 / L4T R35.1）と既存の Docker には手を付けない。**
eMMC には既存の環境があり空きも 3 GB しかないため、PlaceNav 用の Docker デーモンを別に立て、
イメージとデータは追加した NVMe SSD（`/mnt/ssd`）に置く。

```
parrot (x86, Internet)                          Xavier (JetPack 5.0.2 のまま, offline)
 40 arm64 イメージ作成 ─┐                        docker.service（既存, eMMC）… 変更しない
 45 runtime deb 取得 ───┼── LAN ── 45 で導入 ──▶ fstab に SSD / nvidia-container-toolkit
                        │                         docker-placenav.service（SSD, 別ソケット）
 50 配布 ───────────────┘── LAN ──────────────▶ /mnt/ssd/PlaceNav (src, 重み, topomap)
                                                  run_xavier.sh → bench / navigate
```

| 項目 | 内容 |
|---|---|
| 推論イメージ | `dustynv/l4t-pytorch:r35.4.1`（digest 固定）: Ubuntu 20.04, CUDA 11.4, **PyTorch 2.0.0**, torchvision 0.15.1（sm_72）+ ROS Noetic（rospy/msgs/cv_bridge のみ） |
| Xavier に追加するもの | `/etc/fstab` に SSD 1 行、NVIDIA コンテナ runtime の deb 4 つ（R35.1 用、`nvidia-docker2` は入れない）、`/etc/docker/placenav-daemon.json`、`docker-placenav.service` |
| 変更しないもの | 既存の `docker.service`、`/etc/docker/daemon.json`（存在しない）、eMMC 上のイメージ・コンテナ |
| PlaceNav 用 Docker | ソケット `unix:///run/docker-placenav.sock`、データ `/mnt/ssd/docker-placenav`、containerd 名前空間 `placenav`、既定 runtime nvidia |
| 設定値 | IP・ユーザ名・パスは `common.sh`。環境変数で上書き可 |

## 手順（parrot 上、このディレクトリで実行）

```bash
./40_build_image.sh    # linux/arm64 を qemu でビルド（コンパイルなし、数十分）→ ~/xavier_jp512/placenav-xavier.tar.gz
./45_setup_xavier.sh   # deb 取得（sha256 確認）→ Xavier へ転送 → Xavier で sudo 実行（パスワードを聞かれる）
./50_deploy.sh         # PlaceNav 用 Docker へイメージ読込（未読込時のみ）と src/・docker/ を /mnt/ssd/PlaceNav へ rsync
```

`45_setup_xavier.sh` の最後に、PlaceNav 用 Docker の情報（`root=/mnt/ssd/docker-placenav`, `default=nvidia`）と、
既存 Docker が変わっていないこと（`active`, root `/var/lib/docker`）が表示される。

`50_deploy.sh` は parrot 上の `src/model_weights` と `src/topomaps` をそのまま送る。
**先に parrot 側で使うモデルの準備（`prepare_waypoint.py`, `prepare_vpr.py`, 重みのダウンロード）を済ませておくこと。**
`global-feats-*.h5` は送らず、Xavier 上で初回起動時に作り直す。

### 動作確認と推論時間の計測（Xavier 上）

```bash
ssh amsl@192.168.1.246
sudo jetson_clocks   # 計測・走行前に毎回（再起動で戻る）。電力モードは sudo nvpmodel -q で確認
/mnt/ssd/PlaceNav/docker/xavier/run_xavier.sh python3 ../docker/xavier/check_xavier.py   # GPU・cuDNN・cv_bridge（qemu では確認できない分）
/mnt/ssd/PlaceNav/docker/xavier/run_xavier.sh python3 bench_inference.py \
    --pr-models cosplace salad megaloc --wp-models omnivla_edge nomad gnm_large
```

5 Hz で回すには VPR + ウェイポイント推論の合計が 200 ms を十分下回る必要がある。
2026-11-14 の計測（中央値）: CosPlace 21.5 / SALAD 121.8 / MegaLoc 121.1、GNM large 20.9 /
OmniVLA-edge 100.5 / NoMaD 186.0 ms。使う組み合わせは **CosPlace + OmniVLA-edge**（約 122 ms）。
PlaceNav 用 Docker を手で使うときは `docker -H unix:///run/docker-placenav.sock ...`
（既定の `docker` は既存のデーモンを指す）。

### 通信の確認（`link_probe.py`、モデル不要）

parrot から Xavier へのカメラ画像の届き方（頻度・欠落・遅れ）と、計算時間を足した経路点の遅れを測る。

```bash
# parrot（3 つとも起動したまま）
python3 docker/xavier/link_probe.py echo      # Xavier が時計のずれを測るための応答役
python3 docker/xavier/link_probe.py camera    # 実カメラを起動しない場合だけ（1280x720 の生画像 5 Hz）
# Xavier
/mnt/ssd/PlaceNav/docker/xavier/run_xavier.sh python3 ../docker/xavier/link_probe.py measure --work-ms 122
```

1 Gbps では 1280x720 の生画像が欠落なく約 30 ms で届く。100 Mbps だと 1 枚 235 ms かかり欠落する
（2026-11-14 にハブ–Xavier 間の不良ケーブルで 100 Mbps になっていた。Xavier で
`cat /sys/class/net/eth0/speed` が 1000 であること）。

## 実機での走行（parrot で launch、Xavier で navigate.py）

```bash
# 1. parrot: 通常どおり。navigate.py だけ起動しない。path_timeout > 0 でないと launch が拒否する
roslaunch rwrc26_bringup navigation.launch remote_navigation:=true path_timeout:=1.0 \
    wp_model:=omnivla_edge pr_model:=cosplace topomap_dir:=<route> drive_robot:=false
# 2. Xavier: parrot の /placenav_runner のパラメータをそのまま使って navigate.py を起動する
sudo jetson_clocks
/mnt/ssd/PlaceNav/docker/xavier/navigate_xavier.sh
```

- `navigate_xavier.sh` は最初に chrony を確認する。時刻源が parrot でない、またはずれが 5 ms を超える
  （再起動直後に起きる）場合は起動しない。
- route・モデル・各種インデックスは parrot の launch 引数だけで決まる。Xavier 側で指定するものはない。
- parrot の launch を止める（または再起動する）と、Xavier 側の navigate.py も止まる。
- Xavier か LAN が止まると、parrot の TEB は `path_timeout` 秒で経路を失って停止する。
- `remote_navigation:=true` では parrot 側の PlaceNav GUI は出ない（位置推定の DB が Xavier 側にあるため）。rviz は出る。
- topomap は `50_deploy.sh` で Xavier に送ってあること。初回は Xavier で `global-feats-cosplace.h5` を作るので起動に時間がかかる。
- 最初は `drive_robot:=false` で、Xavier のログの `Average runtime` と rviz の経路を確認してから `true` にする。

### 元に戻す

```bash
sudo systemctl disable --now docker-placenav
sudo rm /etc/systemd/system/docker-placenav.service /etc/docker/placenav-daemon.json
sudo rm -rf /mnt/ssd/docker-placenav /mnt/ssd/PlaceNav
# 必要なら: /etc/fstab の SSD の行を削除（バックアップ /etc/fstab.before-placenav.* あり）
# 必要なら: sudo dpkg -r nvidia-container-toolkit libnvidia-container-tools libnvidia-container1 libnvidia-container0
```

## 未検証・注意点

- **R35.4.1 用のイメージを R35.1 のホストで動かす**（CUDA 11.4 は共通、ドライバはホストのものを使用）。
  2026-11-13 に `check_xavier.py` で CUDA・cuDNN・cv_bridge の動作を確認済み。
- 40〜50 と `check_xavier.py`・`bench_inference.py`・`link_probe.py` は parrot と実機 Xavier で実行済み（2026-11-13〜14）。
  `navigate_xavier.sh` と `remote_navigation:=true` での走行は未実施。
- イメージの PyTorch は 2.0.0（x86 イメージは 2.1.2）。x86 で書き出した VPR の TorchScript（`model_weights/vpr/*.pt`）を
  2.0 で読めない場合は、Xavier 上で `prepare_vpr.py --model <名前>` を再実行して書き出し直す
  （上流ソースと重みのキャッシュも Xavier に必要）。
- Xavier の CPU で行う画像の前処理（1280x720 の変換・縮小）の時間は未計測。`Average runtime` で確認する。

## 参考: Xavier を消去して JetPack 5.1.2 を書き込む場合（現在は使わない）

既存環境を消してよくなった場合のみ。`10_download_bsp.sh` → `sudo 20_prepare_rootfs.sh` →
（USB-C で接続しリカバリモードにして）`sudo 30_flash.sh`。**eMMC は全消去される。**
書き込み後は既定の Docker が nvidia runtime を持つため、`45_setup_xavier.sh` は使わず、
`PLACENAV_DOCKER_HOST=unix:///var/run/docker.sock XAVIER_PLACENAV_DIR=/home/amsl/PlaceNav ./50_deploy.sh` のように既定の Docker を指定する。

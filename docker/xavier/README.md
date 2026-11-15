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
                                               run_xavier.sh → check / bench
```

| ファイル | 実行する場所 | 内容 |
|---|---|---|
| `build_image.sh` | parrot | `Dockerfile` から arm64 イメージを qemu でビルドし、`~/xavier_jp512/placenav-xavier.tar.gz` に保存 |
| `setup_xavier.sh` | parrot | NVIDIA コンテナ runtime の deb を取得し、Xavier で `setup_xavier_host.sh` を sudo 実行する（初回のみ） |
| `deploy.sh` | parrot | イメージ（未読込時のみ）と `src/`・`docker/`（重み・topomap を含む）を Xavier に送る |
| `run_xavier.sh` | Xavier | 推論コンテナで任意のコマンドを実行（parrot の ROS master に接続） |
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

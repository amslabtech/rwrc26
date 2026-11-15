# Shared settings for the Xavier scripts. Sourced by the scripts in this directory.
# Override any value through the environment, e.g. XAVIER_IP=192.168.1.10 ./deploy.sh
# shellcheck shell=bash

# Same directory with and without sudo (sudo would otherwise switch $HOME to /root).
INVOKING_HOME=$(getent passwd "${SUDO_USER:-$(id -un)}" | cut -d: -f6)
# Image tarball and runtime debs on parrot (name kept so existing files are found).
WORK_DIR=${WORK_DIR:-${INVOKING_HOME}/xavier_jp512}

# Link between parrot and the Xavier (no Internet on that link).
XAVIER_IP=${XAVIER_IP:-192.168.1.246}
PARROT_IP=${PARROT_IP:-192.168.1.146}
XAVIER_USER=${XAVIER_USER:-amsl}

# Inference image (built on parrot for linux/arm64, then copied to the Xavier).
# The base image is pinned in the Dockerfile.
IMAGE=${IMAGE:-placenav-xavier:r35.4.1}
IMAGE_TAR=${WORK_DIR}/placenav-xavier.tar.gz

# The Xavier keeps its JetPack 5.0.2 (L4T R35.1) install and existing Docker untouched.
# PlaceNav gets its own dockerd whose data, socket and containerd namespace live apart.
XAVIER_SSD=${XAVIER_SSD:-/mnt/ssd}
XAVIER_PLACENAV_DIR=${XAVIER_PLACENAV_DIR:-${XAVIER_SSD}/PlaceNav}
PLACENAV_DOCKER_HOST=${PLACENAV_DOCKER_HOST:-unix:///run/docker-placenav.sock}
# NVIDIA container runtime for L4T R35.1 (JetPack 5.0.2 versions), without nvidia-docker2,
# which would rewrite /etc/docker/daemon.json of the existing Docker.
RUNTIME_DEB_URL=https://repo.download.nvidia.com/jetson/common
RUNTIME_DEBS=(
    "pool/main/libn/libnvidia-container/libnvidia-container0_0.11.0+jetpack_arm64.deb 58f39cd9277a6fd67f036fb452ed81fc1f7cdfa2442023b3e2d81a6fd7a74072"
    "pool/main/libn/libnvidia-container/libnvidia-container1_1.10.0-1_arm64.deb 873e17b348329187d6b7a6669d9e6488bcf652bcda917eb4a3a7f67f7e130cb6"
    "pool/main/libn/libnvidia-container/libnvidia-container-tools_1.10.0-1_arm64.deb 8bafa1a1711c6cbe6116ea06cda476e6a61fcd4c7954b05ef620e29f5f33f8d5"
    "pool/main/n/nvidia-container-toolkit/nvidia-container-toolkit_1.11.0~rc.1-1_arm64.deb e5be517d6cb5a7f61d2b7e8a596f972c890f2453186555c7ef1d4dd9fd844b0a"
)

die() { echo "ERROR: $*" >&2; exit 1; }

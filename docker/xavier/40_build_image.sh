#!/usr/bin/env bash
# [parrot, Internet] Build the arm64 inference image under qemu and save it as a tarball.
set -euo pipefail
source "$(dirname "$0")/common.sh"
PLACENAV_DIR=$(cd "$(dirname "$0")/../.." && pwd)

# qemu-user-static (installed by l4t_flash_prerequisites.sh) registers this handler.
[[ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ]] || die "no arm64 binfmt handler; sudo apt-get install qemu-user-static binfmt-support"

docker buildx build --platform linux/arm64 --load \
    -f "${PLACENAV_DIR}/docker/xavier/Dockerfile" -t "${IMAGE}" "${PLACENAV_DIR}"

# Import check under qemu. Only located here (really imported on the Xavier by
# check_xavier.py): torch & co. need the Jetson GPU driver, and cv_bridge's boost
# module fails to initialise under qemu even in the plain arm64 ros:noetic image.
docker run --rm --platform linux/arm64 -v "${PLACENAV_DIR}:/placenav:ro" "${IMAGE}" python3 -c "
import importlib.util, rospy, h5py, cv2, placenav_viz_msgs.msg
missing = [m for m in ('torch', 'torchvision', 'efficientnet_pytorch', 'diffusers', 'gnm_train', 'cv_bridge')
           if importlib.util.find_spec(m) is None]
assert not missing, missing
print('imports ok (qemu); run check_xavier.py on the Xavier for the rest')"

mkdir -p "${WORK_DIR}"
docker save "${IMAGE}" | gzip -1 > "${IMAGE_TAR}.part"
mv "${IMAGE_TAR}.part" "${IMAGE_TAR}"
ls -lh "${IMAGE_TAR}"
echo "Next: ./50_deploy.sh"

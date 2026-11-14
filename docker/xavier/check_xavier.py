#!/usr/bin/env python3
"""[Xavier, inside the container] Check what cannot be checked under qemu on parrot.

    /mnt/ssd/PlaceNav/docker/xavier/run_xavier.sh python3 ../docker/xavier/check_xavier.py
"""
import numpy as np


def main():
    import torch
    print("torch", torch.__version__, "CUDA available:", torch.cuda.is_available())
    assert torch.cuda.is_available(), "no GPU in the container (nvidia runtime / R35.1 host driver)"
    print("device:", torch.cuda.get_device_name(0))
    x = torch.randn(256, 256, device="cuda")
    torch.testing.assert_close((x @ x).cpu(), x.cpu() @ x.cpu(), rtol=1e-3, atol=1e-3)
    conv = torch.nn.Conv2d(3, 8, 3).cuda()  # exercises cuDNN
    conv(torch.randn(1, 3, 32, 32, device="cuda"))
    print("CUDA matmul + cuDNN conv ok")

    import torchvision, efficientnet_pytorch, diffusers, gnm_train  # noqa: F401
    print("torchvision", torchvision.__version__)

    from cv_bridge import CvBridge
    image = (np.arange(4 * 5 * 3) % 256).astype(np.uint8).reshape(4, 5, 3)
    bridge = CvBridge()
    assert (bridge.imgmsg_to_cv2(bridge.cv2_to_imgmsg(image, "bgr8"), "bgr8") == image).all()
    print("cv_bridge round trip ok")
    print("ALL OK")


if __name__ == "__main__":
    main()

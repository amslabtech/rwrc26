# PlaceNav inference image for Jetson AGX Xavier (JetPack 5.1.2 / L4T R35.4.1).
# Built on parrot with:  ./docker/xavier/build_image.sh   (context: PlaceNav root)
#
# Base: Ubuntu 20.04 + CUDA 11.4 + PyTorch 2.0.0 + torchvision 0.15.1 built for sm_72.
# Only inference dependencies; PlaceNav sources and weights are bind-mounted at run time.
ARG BASE_IMAGE=dustynv/l4t-pytorch:r35.4.1@sha256:76585936ace4d6c7e6bfaf7624c0d33f3fdd2d0ff7f5a06cb20fef1cd01f0d5f
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ROS Noetic (rospy + messages + cv_bridge only; no GUI, no navigation stack).
# ROS Noetic binaries (cv_bridge) and python3-opencv are built against Ubuntu's OpenCV 4.2, so
# use only that one. The base image ships a CUDA OpenCV 4.5.0 as custom debs (opencv-*) that
# own /usr/include/opencv4 and lists NVIDIA's Jetson apt repo, whose OpenCV 4.5.4 would be
# chosen for libopencv-dev. Nothing here uses CUDA OpenCV: remove the former, pin out the latter.
RUN apt-get purge -y opencv-dev opencv-libs opencv-licenses opencv-main opencv-python opencv-scripts \
    && rm -f /usr/local/include/opencv4 /usr/local/lib/python3.8/dist-packages/cv2 \
    && printf 'Package: *opencv*\nPin: origin "repo.download.nvidia.com"\nPin-Priority: -1\n' \
       > /etc/apt/preferences.d/no-nvidia-opencv
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg2 ca-certificates lsb-release \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
       > /etc/apt/sources.list.d/ros1.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       ros-noetic-ros-base ros-noetic-cv-bridge ros-noetic-sensor-msgs ros-noetic-nav-msgs \
       ros-noetic-geometry-msgs ros-noetic-std-msgs python3-opencv git \
    && rm -rf /var/lib/apt/lists/*

# Same pins as the x86 image where an aarch64 cp38 wheel exists. torch/torchvision come
# from the base image and must not be replaced by pip.
RUN python3 -m pip install --no-cache-dir \
       efficientnet-pytorch==0.7.1 einops==0.6.1 diffusers==0.18.2 huggingface-hub==0.23.5 \
       warmup-scheduler==0.3 tqdm==4.64.0 h5py==3.6.0 numpy==1.24.4 scipy==1.10.1 \
       matplotlib==3.7.5 pyyaml rospkg safetensors==0.4.5 omegaconf==2.3.0 \
    && python3 -c "from importlib.metadata import version as v; assert v('torch').startswith('2.0.0'), v('torch')"
# (metadata only: importing torch needs the Jetson GPU driver, absent while building)

# GNM/ViNT checkpoints pickle classes from the official training package. Put its source
# directory on PYTHONPATH (as the x86 legacy editable install does) instead of `pip install -e`:
# with the base image's newer setuptools the editable install left gnm_train unimportable.
ARG DRIVE_ANY_ROBOT_REV=94dd70b02744030d9ed28bb7f5c9fbd241793944
RUN git clone https://github.com/robodhruv/drive-any-robot.git /opt/drive-any-robot \
    && git -C /opt/drive-any-robot checkout --detach ${DRIVE_ANY_ROBOT_REV}
ENV PYTHONPATH=/opt/drive-any-robot/train

COPY src/placenav_viz_msgs/src /tmp/placenav_viz_msgs_ws/src
RUN source /opt/ros/noetic/setup.bash \
    && cd /tmp/placenav_viz_msgs_ws && catkin_make \
    && cp -a devel /opt/placenav_viz_msgs_devel && rm -rf /tmp/placenav_viz_msgs_ws

# OpenBLAS busy-waits after np.dot and burns CPU on small matrices; one thread is enough.
ENV OPENBLAS_NUM_THREADS=1 \
    ROSCONSOLE_FORMAT='[${severity}] [${time}] [${node}]: ${message}' \
    ROSCONSOLE_STDOUT_LINE_BUFFERED=1 \
    PLACENAV_SRC=/placenav/src
COPY docker/xavier/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

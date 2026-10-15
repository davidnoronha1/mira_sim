# Reuse the simulator image; only add the application stack's dependencies.
ARG MIRA_SIM_BASE=ghcr.io/davidnoronha1/mira_sim-gazebo:latest
FROM ${MIRA_SIM_BASE}
USER root
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions python3-scipy python3-opencv python3-venv \
    ros-jazzy-behaviortree-cpp ros-jazzy-vision-msgs ros-jazzy-cv-bridge \
    ros-jazzy-rtabmap-odom ros-jazzy-ros-gz-bridge ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs ros-jazzy-diagnostic-msgs \
    ros-jazzy-ament-cmake-python ros-jazzy-rosidl-default-generators \
    && rm -rf /var/lib/apt/lists/*
# Published bases built before gazebo.Dockerfile gained /opt/mavenv lack it.
RUN [ -x /opt/mavenv/bin/pip ] || { python3 -m venv --system-site-packages /opt/mavenv \
    && /opt/mavenv/bin/pip install --no-cache-dir pymavlink; }
ENV PATH="/opt/mavenv/bin:${PATH}"
# cu128 supports Blackwell; CPU service overrides this to cpu.
# Version pairs: https://pytorch.org/get-started/previous-versions/
ARG TORCH_VARIANT=cu128
RUN /opt/mavenv/bin/pip install --no-cache-dir --index-url https://download.pytorch.org/whl/${TORCH_VARIANT} torch==2.7.1 torchvision==0.22.1 \
    && /opt/mavenv/bin/pip install --no-cache-dir 'numpy<2' 'opencv-python<4.12' ultralytics onnxruntime albumentations
ENV PYTHONPATH=/opt/mavenv/lib/python3.12/site-packages
# GPU visual odometry (mira2_vision vo.backend=cuvslam). RGB-D tracking needs
# CUDA, so the CPU variant skips it and navigation falls back to RTAB-Map.
# Releases: https://github.com/nvidia-isaac/cuVSLAM/releases
ARG CUVSLAM_VERSION=17.0.1
RUN if [ "${TORCH_VARIANT}" != cpu ]; then /opt/mavenv/bin/pip install --no-cache-dir \
    "https://github.com/nvidia-isaac/cuVSLAM/releases/download/v${CUVSLAM_VERSION}/cuvslam-${CUVSLAM_VERSION}%2Bcu12-cp312-abi3-manylinux_2_39_x86_64.whl"; fi

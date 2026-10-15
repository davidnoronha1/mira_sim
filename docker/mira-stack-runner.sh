#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
# Ultralytics writes settings under $HOME/.config, which may be unwritable here.
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-/tmp}"
case "${1:?component required}" in
  build)
    cd /mira_build
    exec colcon --log-base /mira_build/log build \
      --build-base /mira_build/build --install-base /mira_build/install \
      --base-paths /mira/src/dependencies/custom_msgs /mira/src/dependencies/utils \
                   /mira/src/mira2_vision /mira/src/mira2_hardware /mira/src/mira2_actions \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
    ;;
  guided)
    source /mira_build/install/setup.bash
    exec ros2 launch mira2_hardware guided.launch.py simulation:=true \
      config:="${VISION_CONFIG:-/mira/src/mira2_vision/config/guided_front.yaml}" \
      yolo_device:="${YOLO_DEVICE:-CPU}" yolo_model:="${YOLO_MODEL:-best.onnx}" \
      source:="${VISION_SOURCE:-vo}" start_x:="${START_X:-7}" start_y:="${START_Y:--10}" \
      auto_arm:="${AUTO_ARM:-true}" visualize:="${VISUALIZE:-false}" vo_backend:="${VO_BACKEND:-auto}" \
      detectors:="$([ "${SIM_DETECTOR:-gazebo}" = gazebo ] && echo gazebo_boxes)"
    ;;
  actions)
    source /mira_build/install/setup.bash
    python3 /workspace/mira-stack/wait-mira-target.py "${TARGET_CLASS:-dock}" "${ACTION_WAIT:-120}"
    exec ros2 launch mira2_actions mira2_actions_launch.py use_sim_time:=true \
      target_class:="${TARGET_CLASS:-dock}"
    ;;
  *) echo "Unknown component: $1" >&2; exit 2 ;;
esac

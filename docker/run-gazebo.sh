#!/usr/bin/env bash
# Runs Gazebo (and optionally ros_gz_bridge) using system-installed Gazebo
# if available; falls back to Docker if it fails.

set -e

HOST_WORLD="$1"
DOCKER_WORLD="$2"
HOST_BRIDGE_CONFIG="$3"
DOCKER_BRIDGE_CONFIG="$4"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -z "$HOST_WORLD" ]; then
  echo "Usage: $0 <host_world_path> [docker_world_path] [host_bridge_config] [docker_bridge_config]" >&2
  exit 1
fi

if [[ "$HOST_WORLD" != /* ]]; then
  HOST_WORLD="$REPO_ROOT/$HOST_WORLD"
fi

if [ -z "$DOCKER_WORLD" ]; then
  DOCKER_WORLD="$HOST_WORLD"
fi

if [ -n "$HOST_BRIDGE_CONFIG" ] && [[ "$HOST_BRIDGE_CONFIG" != /* ]]; then
  HOST_BRIDGE_CONFIG="$REPO_ROOT/$HOST_BRIDGE_CONFIG"
fi

if [ -n "$HOST_BRIDGE_CONFIG" ] && [ -z "$DOCKER_BRIDGE_CONFIG" ]; then
  DOCKER_BRIDGE_CONFIG="$HOST_BRIDGE_CONFIG"
fi

# Reset bridge log file
rm -f /tmp/ros_gz_bridge.log 2>/dev/null || true
touch /tmp/ros_gz_bridge.log 2>/dev/null || true

# Check if user explicitly forced docker
USE_SYSTEM_GZ="${USE_SYSTEM_GZ:-1}"
if [ "${USE_DOCKER:-0}" = "1" ] || [ "${MIRA_USE_DOCKER:-0}" = "1" ]; then
  USE_SYSTEM_GZ=0
fi

run_docker() {
  local reason="$1"
  echo "⚠️  WARNING: $reason"
  echo "🐳 Falling back to Docker container [${GZ_SERVICE:-mira_sim}]..."

  local gz_svc="${GZ_SERVICE:-mira_sim}"
  docker compose up --no-recreate -d "$gz_svc"

  # Start bridge in Docker if requested
  if [ -n "$DOCKER_BRIDGE_CONFIG" ]; then
    echo "🐳 Starting ros_gz_bridge and underwater_camera_node inside Docker container..."
    docker compose exec "$gz_svc" pkill -9 -f parameter_bridge 2>/dev/null || true
    docker compose exec "$gz_svc" pkill -9 -f underwater_camera 2>/dev/null || true
    (
      sleep 2
      docker compose exec "$gz_svc" bash -c "source /opt/ros/jazzy/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash; (ros2 run bridge underwater_camera_node 2>/dev/null || ros2 run sauvc_sim underwater_camera_node.py 2>/dev/null) & exec ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$DOCKER_BRIDGE_CONFIG"
    ) > /tmp/ros_gz_bridge.log 2>&1 &
    DOCKER_BRIDGE_PID=$!
    trap 'kill $DOCKER_BRIDGE_PID 2>/dev/null || true; docker compose exec "$gz_svc" pkill -9 -f parameter_bridge 2>/dev/null || true; docker compose exec "$gz_svc" pkill -9 -f underwater_camera 2>/dev/null || true' EXIT INT TERM
  fi

  exec docker compose exec "$gz_svc" bash -c "source /tmp/gz-render-env.sh 2>/dev/null; exec gz sim ${GZ_ARGS:--v3 -r} $DOCKER_WORLD"
}

if [ "$USE_SYSTEM_GZ" != "1" ]; then
  run_docker "System Gazebo disabled by configuration (USE_SYSTEM_GZ=$USE_SYSTEM_GZ)."
  exit 0
fi

# Source ROS Jazzy if available to get gz and ros2 in PATH
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
if [ -f "$REPO_ROOT/install/setup.bash" ]; then
  source "$REPO_ROOT/install/setup.bash"
fi

# Check if gz command is available
if ! command -v gz >/dev/null 2>&1; then
  run_docker "System Gazebo ('gz' binary) not found in PATH."
  exit 0
fi

# Check / build ardupilot_gazebo plugin
if [ ! -f "$REPO_ROOT/src/ardupilot_gazebo/build/libArduPilotPlugin.so" ]; then
  echo "⚙️  Building ardupilot_gazebo plugin for system Gazebo..."
  if ! (cmake -B "$REPO_ROOT/src/ardupilot_gazebo/build" -S "$REPO_ROOT/src/ardupilot_gazebo" -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
        cmake --build "$REPO_ROOT/src/ardupilot_gazebo/build" -j"$(nproc)"); then
    run_docker "Failed to build ardupilot_gazebo plugin on host."
    exit 0
  fi
fi

# Configure system Gazebo environment
export GZ_SIM_SYSTEM_PLUGIN_PATH="$REPO_ROOT/src/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/src/bluerov2_gz/models:$REPO_ROOT/src/bluerov2_gz/worlds:$REPO_ROOT/worlds:$REPO_ROOT/src/common_resources/data/object_files:$REPO_ROOT/src/sauvc_sim/models:$REPO_ROOT/src/sauvc_sim/worlds:$REPO_ROOT/src/ardupilot_gazebo/models:$REPO_ROOT/src/ardupilot_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_CONFIG_PATH="/usr/share/gz:/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz${GZ_CONFIG_PATH:+:$GZ_CONFIG_PATH}"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# Configure rendering engine
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
  export GZ_RENDER_ENGINE=ogre2
elif [ -e /dev/dri ]; then
  export GZ_RENDER_ENGINE=ogre2
else
  export LIBGL_ALWAYS_SOFTWARE=1
  export GZ_RENDER_ENGINE=ogre
fi

echo "🚀 Starting system-installed Gazebo..."
echo "   World: $HOST_WORLD"
echo "   Rendering engine: ${GZ_RENDER_ENGINE:-default}"

# Start host ros_gz_bridge if configured
HOST_BRIDGE_PID=""
HOST_UNDERWATER_PID=""
if [ -n "$HOST_BRIDGE_CONFIG" ] && [ -f "$HOST_BRIDGE_CONFIG" ]; then
  if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
    echo "🚀 Starting ros_gz_bridge on host (co-located with system Gazebo)..."
    (
      sleep 2
      exec ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$HOST_BRIDGE_CONFIG"
    ) > /tmp/ros_gz_bridge.log 2>&1 &
    HOST_BRIDGE_PID=$!

    # Launch underwater_camera_node to process dry camera topics into realistic underwater imagery
    if ros2 pkg prefix bridge >/dev/null 2>&1 || ros2 pkg prefix sauvc_sim >/dev/null 2>&1; then
      echo "🌊 Starting underwater_camera_node (publishing /camera_front/image_raw with underwater effects)..."
      (
        sleep 3
        if ros2 pkg prefix bridge >/dev/null 2>&1; then
          exec ros2 run bridge underwater_camera_node
        else
          exec ros2 run sauvc_sim underwater_camera_node.py
        fi
      ) >> /tmp/ros_gz_bridge.log 2>&1 &
      HOST_UNDERWATER_PID=$!
    fi

    trap 'kill $HOST_BRIDGE_PID $HOST_UNDERWATER_PID 2>/dev/null || true' EXIT INT TERM
  else
    echo "⚠️  WARNING: ros_gz_bridge not found on host."
  fi
fi

# Auto-apply water_world.config GUI camera layout if viewing sauvc / water_world
if [[ "$HOST_WORLD" == *"water_world"* || "$HOST_WORLD" == *"sauvc"* ]] && [ -f "$REPO_ROOT/src/sauvc_sim/config/water_world.config" ] && [[ "${GZ_ARGS:-}" != *"--gui-config"* ]]; then
  GZ_ARGS="${GZ_ARGS:--v3 -r} --gui-config $REPO_ROOT/src/sauvc_sim/config/water_world.config"
fi

START_TIME=$(date +%s)
set +e
gz sim ${GZ_ARGS:--v3 -r} "$HOST_WORLD"
GZ_EXIT_CODE=$?
set -e
END_TIME=$(date +%s)
DURATION=$(( END_TIME - START_TIME ))

# If host Gazebo crashed / failed quickly (< 4 seconds), clean up host bridge and fallback to Docker
if [ $GZ_EXIT_CODE -ne 0 ] && [ $GZ_EXIT_CODE -ne 130 ] && [ $DURATION -lt 4 ]; then
  if [ -n "$HOST_BRIDGE_PID" ]; then
    kill "$HOST_BRIDGE_PID" "$HOST_UNDERWATER_PID" 2>/dev/null || true
    trap - EXIT INT TERM
  fi
  run_docker "System Gazebo failed to start (exit code: $GZ_EXIT_CODE after ${DURATION}s)."
  exit 0
fi

exit $GZ_EXIT_CODE

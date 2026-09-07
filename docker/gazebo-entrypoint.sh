#!/usr/bin/env bash
# Auto-detects GPU availability inside the container and configures Gazebo's
# rendering accordingly, so the same image works on modest/integrated
# hardware (software rendering fallback) and on a real GPU (hardware
# rendering) without any manual flags.
#
# The decision is also written to $RENDER_ENV_FILE so that `docker compose
# exec` sessions (which do NOT inherit env vars exported here - they only
# inherit the container's original env) can `source` it before launching
# `gz sim`. Without this, exec'd `gz sim` processes fall back to gz-sim's
# own default (hardware ogre2) even on GPU-less hosts, and camera/RGBD
# sensors silently produce no frames (topics exist but never publish).
set -e

RENDER_ENV_FILE=/tmp/gz-render-env.sh

if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
  echo "[gazebo-entrypoint] NVIDIA GPU detected via nvidia-smi, using hardware rendering (ogre2)"
  echo "export GZ_RENDER_ENGINE=ogre2" > "$RENDER_ENV_FILE"
elif [ -e /dev/dri ]; then
  echo "[gazebo-entrypoint] /dev/dri present, using hardware rendering (ogre2)"
  echo "export GZ_RENDER_ENGINE=ogre2" > "$RENDER_ENV_FILE"
else
  echo "[gazebo-entrypoint] No GPU device found, falling back to software rendering (llvmpipe/ogre)"
  {
    echo "export LIBGL_ALWAYS_SOFTWARE=1"
    echo "export GZ_RENDER_ENGINE=ogre"
  } > "$RENDER_ENV_FILE"
fi

source "$RENDER_ENV_FILE"
exec "$@"

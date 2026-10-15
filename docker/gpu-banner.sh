#!/usr/bin/env bash
# Loud banner stating whether a bringup runs on the NVIDIA GPU, and why not.
# Usage: gpu-banner.sh <compose service chosen by the Makefile> [tmux session]
service="${1:?service required}"

if nvidia-smi -L >/dev/null 2>&1; then
  gpu="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)"
  driver=1
else
  driver=0
fi
docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"' && runtime=1 || runtime=0

if [ "$service" = mira-sim-gpu ]; then
  colour='\033[1;42;30m'
  title="GPU ENABLED: ${gpu:-NVIDIA (forced by MIRA_GPU=1)}"
  lines=("Gazebo rendering, YOLO (CUDA) and GPU-capable nodes use the NVIDIA GPU."
         "Service: $service.  Force CPU with MIRA_GPU=0.")
  if [ "$driver" = 0 ] || [ "$runtime" = 0 ]; then
    colour='\033[1;41;97m'
    title="GPU FORCED BUT NOT USABLE"
    lines=("driver ok: $driver   docker nvidia runtime: $runtime"
           "The GPU container will fail to start. Unset MIRA_GPU to fall back to CPU.")
  fi
else
  colour='\033[1;43;30m'
  title="GPU NOT AVAILABLE - RUNNING ON CPU"
  if [ "${MIRA_GPU:-}" = 0 ]; then
    reason="Disabled by MIRA_GPU=0."
  elif [ "$driver" = 0 ]; then
    reason="No working NVIDIA driver (nvidia-smi failed)."
  elif [ "$runtime" = 0 ]; then
    reason="Found $gpu, but Docker has no nvidia runtime."
    hint="Install nvidia-container-toolkit, then: sudo nvidia-ctk runtime configure --runtime=docker"
  else
    reason="Service overridden to $service."
  fi
  lines=("$reason" ${hint:+"$hint"}
         "Gazebo uses software/iGPU rendering; YOLO and visual odometry run on CPU."
         "Expect low camera rates; HEADLESS=1 and VISUALIZE=0 help.")
fi

bar="$(printf '%*s' 104 '' | tr ' ' '#')"
printf "\n${colour}%s\033[0m\n" "$bar"
printf "${colour}##  %-98s##\033[0m\n" "$title" "" "${lines[@]}"
printf "${colour}%s\033[0m\n\n" "$bar"

# Optional: keep the verdict visible in a tmux session's status bar.
if [ -n "${2:-}" ] && tmux has-session -t "$2" 2>/dev/null; then
  case "$title" in
    GPU\ ENABLED*) style='bg=green,fg=black,bold' ;;
    *) style='bg=red,fg=white,bold' ;;
  esac
  tmux set-option -t "$2" status-right-length 80 >/dev/null
  tmux set-option -t "$2" status-right "#[$style] ${title} " >/dev/null
fi

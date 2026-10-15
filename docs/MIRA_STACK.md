# Run Mira's vision, hardware and actions packages

Keep `mira_sim` and `mira` as sibling checkouts, or set `MIRA_ROOT` to the Mira
checkout. The integrated target uses Mira's actual source, not copies of its
packages. It preserves the standalone `make bringup-vision` VO testing workflow.

```sh
make mira-stack-build                       # image dependencies + selected colcon packages
make mira-stack-check                       # resolve compose and print commands, no vehicle startup
make bringup-mira                           # start mira-stack tmux session, including one demo
# or:
make bringup-mira HEADLESS=1 RUN_ACTIONS=0    # navigation only; start demo after seeing a target
make mira-actions TARGET_CLASS=dock
make mira-stack-down
```

`make bringup-mira` runs five tmux windows: SITL, Gazebo, sensor/TF bridge,
Mira GUIDED stack, and (unless `RUN_ACTIONS=0`) the one-shot actions demo.
Attach with `tmux attach -t mira-stack`. Startup returns without attaching.
Do not run the old `bringup-vision`, `goal-bridge`, waypoint sender or RC master
alongside it; these would compete for goals, arming, ports or odometry.
Existing known simulator tmux sessions cause an explicit startup failure.

The guided window launches `mira2_hardware guided.launch.py simulation:=true`.
This includes the `mira2_vision` camera worker (YOLO, shared registered depth,
VO and obstacle consumers), RTAB-Map, and the hardware goal bridge. The old
`vision_nav/vo.sh`, `nav.launch.py`, `vision_bridge.py`, `goal_bridge.py` and
`obstacle_distance.py` are not started. Only `vision_nav/bridge.sh` is retained
for Gazebo sensor topics, clock and calibrated static camera TF. All ROS
processes run inside the same simulator container and use UDP DDS transport.

The actions window waits up to `ACTION_WAIT=120` wall-clock seconds for a fresh
localized YOLO detection and EKF feedback, then launches the single Mira demo.
It does not inject a ground-truth target or move while waiting. Select an actual
model label using `TARGET_CLASS`; the default is `dock`. If the model
cannot detect that rendered object, the window reports a timeout without
sending a goal. Change `VISION_CONFIG` to a config path inside `/mira` to select
another model or perception profile. All three packages use the same RGB-D and
frame conventions as hardware.

`AUTO_ARM=true` enables auto-arming **only in explicit simulation mode**.
Set `AUTO_ARM=false` to arm manually. `VISION_SOURCE=gt` compares navigation
against simulator truth and skips RTAB-Map; normal operation uses `vo`.
`VISION_MAP=sauvc|sauvc_quali|tacc` picks the matching world and spawn offset.
`GZ_SERVICE=mira_sim|mira-sim-gpu` selects rendering as in the existing targets.
These select dedicated `mira-stack` / `mira-stack-gpu` Compose services.
The original simulator services and image remain unchanged. Both images reuse
the existing Gazebo/ROS/RTAB-Map base. `mira-stack-gpu` adds CUDA 12.8 PyTorch
(2.7.1 / torchvision 0.22.1, including Blackwell support) and runs `best.pt` on
GPU 0. The required CUDA libraries come with the PyTorch wheels; the host provides
the NVIDIA driver. No separate TensorRT or GPU ONNX runtime is installed.
`mira-stack` retains CPU PyTorch and `best.onnx`. Their image tags are separate
(`mira-sim-stack:cu128` / `mira-sim-stack:cpu`), so builds cannot overwrite each
other. Physical camera drivers are not needed for simulated RGB-D input.

```sh
make mira-stack-build GZ_SERVICE=mira-sim-gpu
make bringup-mira GZ_SERVICE=mira-sim-gpu
```

The Makefile selects the NVIDIA service automatically when a GPU is available.
The launch arguments `yolo_device` / `yolo_model` override the perception profile;
the GPU service supplies `GPU` / `best.pt`. Missing CUDA support causes an explicit
inference error rather than silently running the requested GPU model on CPU.

To build the image directly (the Make target also builds the ROS workspace):

```sh
docker compose -f docker-compose.yml -f docker-compose.mira.yml build mira-stack
```

Both new services belong to the opt-in `mira` profile and can be selected
explicitly. Run just one rendering service at a time.

The `docker-compose.mira.yml` overlay mounts Mira read-only at `/mira`. Build,
install and logs live in `.mira-stack/` in this repository; host Mira artifacts
are untouched. The selected build includes `custom_msgs`, `utils`,
`mira2_vision`, `mira2_hardware` and `mira2_actions`, excluding physical-camera
and legacy mission packages. Re-run `make mira-stack-build` after source changes
and restart the stack. It can build without host ROS/Jazzy/Python 3.12.

The underlying simulator image and SITL must already contain the camera/model
assets and ArduSub external-navigation/OA firmware described in VISION_NAV.md.
The overlay adds application dependencies; it does not rebuild ArduSub or
change firmware parameters. The integration does not validate a model against
the rendered pool automatically.

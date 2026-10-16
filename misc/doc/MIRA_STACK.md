# Connecting Mira to the simulator

Start the simulator independently:

```bash
cd mira_sim
python3 bringup.py up --map sauvc
python3 bringup.py attach
```

This starts only two containers: Gazebo and SITL. The Gazebo container also runs the ROS sensor/TF bridge. Tmux session `mira-sim` has `gazebo`, `sitl` and `bridge` windows, with exited panes retained for log inspection.

Then, in the normal Mira environment (the original `mira`/`mira-nogpu` container, or a configured ROS workspace):

```bash
cd mira
make guided-sim
# Equivalent:
ros2 launch mira2_hardware guided_sim.launch.py
```

Build Mira normally after adding the launch file (`make build`, or select the required packages with the existing build command). There is no simulator stack image, stack Compose overlay, or separate stack build.

The simulation launch includes the existing `guided.launch.py`. It supplies:

- Guided-master MAVLink: `udpin:127.0.0.1:14557`.
- External-navigation MAVLink: `udpin:127.0.0.1:14555`.
- Obstacle-distance MAVLink: `udpin:127.0.0.1:14558`.
- RGB, depth, camera-info and IMU topics published by the simulator bridge.
- Simulation time from `/clock` and matching DDS transport.
- Optional Gazebo bounding-box detections, with automatic SLAM backend selection.

The MAVLink addresses are local UDP listeners: SITL sends to these ports. The existing Mira containers use host networking, so their localhost is the simulator host's localhost. `bind_ip` defaults to `127.0.0.1`. Both ROS environments must use the same `ROS_DOMAIN_ID` (default 0).

`mira2_vision` already accepts ROS image input through `image_source: ros2://<topic>`; its navigation launch constructs this from `rgb_topic`. No camera adapter or extra vision implementation is needed.

## Optional settings

```bash
make guided-sim ARGS="pose_source:=gt"
make guided-sim ARGS="detectors:=yolo slam_algorithm:=rtabmap"
make guided-sim ARGS="auto_arm:=true"
make guided-sim ARGS="rgb_topic:=/custom/rgb depth_topic:=/custom/depth camera_info_topic:=/custom/camera_info"
```

Auto-arming is off by default, as in the regular guided launch. Actions are launched separately using Mira's normal mission launch; connecting the stack does not start a demo.

`start_x`/`start_y` default to zero, establishing a local navigation origin at startup. To align with Gazebo's global coordinates (particularly when comparing ground truth), use the start coordinates printed by `bringup.py`: SAUVC competition `(7, -10)`, qualification `(11, -11.5)`, TACC `(0, 0)`.

Optical flow additionally needs the pool-floor settings. Make them available in your Mira environment and supply their path:

```bash
make guided-sim ARGS="pose_source:=optical_flow flow_scene_config:=/path/to/sauvc_sim/config/optical_flow.yaml"
```

Both SAUVC layouts share that file. TACC has no floor-flow configuration. The simulator does not mount itself into Mira; copy or mount optional scene settings when needed.

## Independent lifecycle

Stop/restart the guided launch without stopping Gazebo or SITL. `python3 bringup.py down` only stops simulator containers and its tmux session; it never stops Mira.

`python3 bringup.py build` rebuilds the simulator images. Mira's image and workspace use their normal build commands. CPU SLAM needs `ros-jazzy-rtabmap-odom`; GPU SLAM can use cuVSLAM when installed in Mira's environment, otherwise `slam_algorithm:=auto` selects the available backend.

ArduSub loads global `config/ardupilot/frame.parm`, Mira's vehicle `external_nav.parm`, global `config/ardupilot/sim.parm`, then the competition's `config/ardusub.parm`. Competition settings live inside `src/sauvc_sim/` or `src/tacc_sim/`. SITL data persists in `var/ardupilot_vision`.

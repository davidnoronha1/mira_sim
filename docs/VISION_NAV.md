# Vision Navigation

GPS-free ArduSub: horizontal position comes from the bluerov2_heavy's front
RGB-D camera via RTAB-Map visual odometry, and the vehicle flies local NED
waypoints in GUIDED mode.

```
gz rgbd_camera ──bridge.sh──► /vision/{image,depth,camera_info}
gz OdometryPublisher ───────► /vision/gt_odom        vo.sh (rtabmap) ─► /vision/vo_odom
                                        └───────┬───────┘
                              vision_bridge.py --source gt|vo
                                        │ MAVLink ODOMETRY, udp 14555
                              ArduSub SITL (ardupilot-sitl-vision)
                                        ▲ udp 14556
                              waypoints.py (GUIDED, SET_POSITION_TARGET_LOCAL_NED)
```

## What feeds ArduSub (and what doesn't)

With the default `VISION_SOURCE=vo`, ArduSub's horizontal position comes only
from RTAB-Map running on the rendered RGB-D images. Gazebo's ground-truth pose
(`/vision/gt_odom`) never reaches ArduSub. It is only drawn in RViz and used
by `compare_odom.py`. ArduSub's IMU, compass and barometer are simulated
from the physics over the SITL JSON link, as on a real vehicle. GPS is off.
`VISION_SOURCE=gt` swaps the truth in, but only for debugging the
ArduPilot/GUIDED side.

## Maps

`make bringup-vision` uses the SAUVC pool by default (`VISION_MAP=sauvc`,
`sauvc25.world`, AUV starts in the starting zone at x=11, y=-11.5 facing
north). `VISION_MAP=tacc` selects the TACC pipeline world (start at the origin).

VO starts at zero wherever the AUV starts, so vision_bridge shifts it by the
map's known start position (`--start-enu`). ArduSub's local frame is then the
Gazebo world frame: goals and waypoint files use plain pool coordinates.

## Click-to-go in RViz

`make bringup-vision` opens RViz with `vision_nav.rviz`:

- **World objects** (gates, mat, starting zone, pool walls/floor, water
  surface) are drawn from the world SDF by `scene_markers.py`, with labels.
  Objects that are commented out in the world file don't appear in either view.
- **AUV**: solid = where ArduSub's EKF thinks it is (what it steers by),
  translucent green = Gazebo truth. Paths: red = EKF, green = truth.
- **2D Goal Pose** (toolbar): click a spot and drag for heading. `goal_bridge.py`
  switches ArduSub to GUIDED, arms it, and flies there at `VISION_DEPTH`
  (default 1 m; e.g. `make bringup-vision VISION_DEPTH=2`).
- **The goal is also highlighted in the Gazebo GUI**: magenta sphere at the
  goal, a pole up to the surface and a heading line. These are GUI-only
  markers, so the simulated cameras (and VO) can't see them.
- **Orange dots** = the obstacle ranges sent to ArduSub (one per sector).
- Grid = water surface, 1 m cells. Frame `odom` = Gazebo world: x = east,
  y = north.
- From a terminal (the tmux `shell` window), with a chosen depth (z < 0):
  `ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 6, y: -6, z: -1.5}, orientation: {w: 1}}}"`

## Recording a demo video

`make record-demo` records the click-to-go flow to
`recordings/vision_nav_demo.mp4`: the Gazebo GUI (left) and RViz (right) side
by side, a real "2D Goal Pose" mouse drag in RViz, and ArduSub flying there
around obstacles. It runs everything on a private Xvfb display (headless is
fine) and replaces `bringup-vision`, so `make bringdown` first.

- Host needs `Xvfb`, `xdotool`, `x11-utils` and `ffmpeg`.
- Default goal: just short of the SAUVC gate, straight through the orange
  flare, so the planner has to bend around it. Change it with
  `GOAL="x y heading_deg"` (Gazebo world coordinates).
- `SPEEDUP=2` speeds the video up (useful with software rendering, where the
  sim runs at ~0.3x real time); `VISION_SOURCE`, `VISION_DEPTH` as above.
- Layouts: `record_gui.config` (Gazebo) and `record.rviz` (top-down RViz view,
  which the script uses to turn the goal into a mouse position).

## Object avoidance (ArduPilot-native)

Goals are not flown in a straight line any more: ArduSub's own object
avoidance path planner (BendyRuler, `OA_TYPE 1`) bends GUIDED legs around
whatever the depth camera sees. This is ArduPilot's
[depth-camera obstacle avoidance](https://ardupilot.org/copter/docs/common-realsense-depth-camera.html)
setup, ported to ArduSub:

```
front RGB-D camera (the same one VO uses)
  -> obstacle_distance.py (port of d4xx_to_mavlink.py: every pixel projected
     into the level body frame using the camera mount from TF; points within
     +/-0.3 m of the AUV's height -> nearest per sector, 72 sectors)
  -> MAVLink OBSTACLE_DISTANCE (udp 14558)
  -> ArduSub proximity (PRX1_TYPE 2) -> object database -> BendyRuler
```

One camera serves both jobs. It is pitched 25° down: the floor fills the
lower ~2/3 of the image for VO, and the top edge sits ~2° above horizontal,
so obstacles at the AUV's height stay in view at any range. Steeper (35°)
gave VO a little more texture but hid obstacles at body height beyond ~2 m.
The pitch lives in `model.sdf` and `bridge.sh` (`CAMERA_PITCH`), and
obstacle_distance.py reads it from TF.

Stock ArduSub can't do this. The OA planner and the proximity library exist in
ArduPilot and are wired into Copter/Rover, but ArduSub never creates the
planner (no `OA_` params) and never initialises or updates proximity, so
`PRX`/`AVOID_` settings do nothing. `docker/ardusub_object_avoidance.patch`
(31 lines, applied in `docker/ardupilot.Dockerfile`) does the Copter-style
wiring: `AC_WPNav_OA` instead of `AC_WPNav`, the `OA_` parameter group, and
proximity init + a 200 Hz update task. Rebuild with
`docker compose build ardupilot-sitl` (30-45 min).

Pool-sized tuning in `ardusub_vision.parm`: `OA_MARGIN_MAX 0.5` (AUV half-width
0.29 m + position error; the gate has 0.75 m either side of its centre),
`OA_BR_LOOKAHEAD 3` (default 15 m sees pool walls everywhere),
`OA_DB_BEAM_WIDTH 1.2` (one ray; the default 5 deg inflates thin posts),
`PRX_FILT 1`.

Limits: BendyRuler is reactive. It only knows what the camera has seen (database
items expire after `OA_DB_EXPIRE` s), searches horizontally only, and won't
make the AUV go *through* a gate on its own. A goal on the far side of the
gate takes the shortest clear path, which may be around it. To force a gate
pass, give it a waypoint in the gate opening at a depth below the top bar
(0.7 m).

## Pieces

| File | Role |
|---|---|
| `docker/ardusub_vision.parm` | EKF3 on, GPS off, `VISO_TYPE=1`, XY pos/vel from ExtNav, Z from baro, yaw from compass, declination pinned |
| `ardupilot-sitl-vision` (compose) | SITL with the overlay above; own `./ardupilot_vision` data dir so params don't leak into the normal SITL |
| `src/vision_nav/bridge.sh` | ros_gz_bridge for camera + GT odometry + `/clock`, static TF `base_link → front_camera_depth_optical` |
| `src/vision_nav/vo.sh` | `rtabmap_odom rgbd_odometry`; extra args are passed through (e.g. `-p Reg/Strategy:="'1'"`) |
| `src/vision_nav/vision_bridge.py` | odometry → MAVLink `ODOMETRY`, sets EKF origin, aligns heading |
| `src/vision_nav/waypoints.py` | waits for EKF position, GUIDED, arm, flies a YAML waypoint list |
| `src/vision_nav/compare_odom.py` | prints VO error vs ground truth once per second |
| `src/vision_nav/obstacle_distance.py` | depth image → MAVLink `OBSTACLE_DISTANCE` (udp 14558) + `/vision/obstacles` LaserScan |
| `docker/ardusub_object_avoidance.patch` | enables the OA path planner + proximity in ArduSub |
| `src/vision_nav/goal_bridge.py` | RViz `/goal_pose` → GUIDED target; publishes EKF pose/path and truth path for RViz (MAVLink udp 14557) |
| `src/vision_nav/scene_markers.py` | world SDF → RViz markers (`/vision/scene`); AUV mesh at EKF and truth poses (`/vision/auv`) |
| `src/vision_nav/vision_nav.rviz` | RViz layout: truth vs EKF, goal, camera image, 2D Goal Pose tool |

The `bluerov2_heavy` model (bluerov2_gz fork) carries the
`OdometryPublisher` plugin and sets `gz_frame_id` on `front_camera_depth`.

## Frames

- Gazebo world is ENU, bodies are FLU. ArduPilot wants NED / FRD.
- `gt`: world ENU → NED directly (north = Gazebo +y, east = +x).
- `vo`: rtabmap's odom frame is FLU anchored at the start pose, so its FRD is
  treated as "NED with north = initial heading".
- For both, the first sample is rotated about the vertical so its heading
  matches ArduSub's AHRS yaw (logged as `Aligned ... rotate N deg`), once
  that yaw has been steady for 3 s (the EKF reports yaw 0 for a moment while
  it initialises). For `gt` the rotation should be ~0; anything large means a
  frame bug. For `vo` it equals the vehicle's true heading at startup.
- The EKF origin is the spawn point, so waypoints are metres from spawn.

## ArduSub gotchas (both handled in `ardusub_vision.parm`)

- **`AHRS_EKF_TYPE` is 10 by default here.** The ardupilot_gazebo plugin sends
  `no_time_sync=true`, and SITL reacts by defaulting to simulator ground-truth
  attitude/position (type 10). External nav is then silently ignored:
  `EKF_STATUS_REPORT` variances read exactly 0. The overlay forces EKF3
  (`AHRS_EKF_TYPE 3`), which is safe because the model runs `lock_step`.
- **Declination.** Without GPS, `COMPASS_AUTODEC` only applies once the EKF has
  a location, i.e. after vision_bridge sets the origin, which is after the
  heading initialised. The EKF yaw then swings by the declination (4.7° here)
  over a few minutes and drifts away from the aligned VO frame. The overlay pins
  `COMPASS_DEC` for the compose `--custom-location` and disables autodec.
  If you change the location, read the new value: let autodec run once, then
  `param show COMPASS_DEC`.

## Verified results (2026-10-07, RTX PRO 6000, tacc.world, `tacc_square`)

| | Ground truth source | VO source |
|---|---|---|
| Waypoints reached | 6/6 | 6/6 |
| Final position error (Gazebo truth) | ~2 cm | ~13 cm |
| Max VO xy error vs truth during flight | 9 cm (VO logged only) | 34 cm |
| VO lost frames | 0 | 0 (plus 1 at startup, by design) |
| VO rate / latency | 30 Hz / ~26 ms | 30 Hz / ~26 ms |

RViz-style goal (2 m east, 3 m north, heading north) on VO only: reached
within 6 cm of the target by Gazebo truth, heading 358°.

SAUVC pool, VO only: about 165-240 inliers per frame (the pool is visually plainer
than TACC; the lane-line floor texture carries it), no lost frames. A ~28 m
run across the pool ended 1.7 m from the goal by Gazebo truth while the EKF
believed it was 0.27 m away, i.e. ~5% drift over distance. Short hops are
accurate; long open-water legs drift. To reduce that, try
`vo.sh -p Reg/Strategy:="'2'"` (visual + depth ICP), or pitch the camera
down so more of the floor texture is in view.

## Bring-up checklist

1. `docker compose build mira-sim-gpu` (the GHCR image lacks rtabmap/pymavlink).
2. `make bringup-vision VISION_SOURCE=gt`
   - `vision_bridge` window: `EKF origin set`, `Aligned ...`, `sent ~30` per second.
   - QGC: no GPS, EKF happy, vehicle appears on the map at the origin.
3. `make waypoints` – should fly the square in Gazebo.
4. `make bringdown`, then `make bringup-vision VISION_SOURCE=vo`.
   - Check rate: `docker compose exec mira-sim-gpu bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic hz /vision/vo_odom"` (want ≥15 Hz).
   - `make compare-odom` while moving the vehicle (QGC joystick or waypoints).
5. `make waypoints` again; compare the final Gazebo pose with the last waypoint.

## Troubleshooting

- **RViz / Gazebo GUI "could not connect to display"**: the X cookie is
  written by `docker/x11-setup.sh` to `/tmp/.mira-sim.xauth` (every GUI make
  target runs it). The compose file refuses to start the sim container if that
  file is missing, rather than letting Docker create a root-owned directory in
  its place, so run it through `make` or `make check-x11` first. A leftover
  root-owned `/tmp/.docker.xauth` directory from older versions is harmless
  and no longer used.
- **No images / 0 Hz**: rendering fell back to software (`/tmp/gz-render-env.sh`
  says `ogre`). Use the `mira-sim-gpu` service (`MIRA_GPU=1`).
- **rtabmap "Did not receive data"**: check the image frame with
  `ros2 topic echo --once /vision/camera_info --field header`. It should be
  `front_camera_depth_optical`. Gazebo logs a harmless "gz_frame_id ... not
  defined in SDF" warning (the model file is SDF 1.6) but still applies it.
- **EKF variances all exactly 0 / vision ignored**: `AHRS_EKF_TYPE` is not 3,
  see the gotchas above.
- **`mavproxy.py: No such file or directory` in the SITL container**: the
  compose `PATH` must include `/opt/mavenv/bin` (fixed in docker-compose.yml).
- **`dropped` counts rising / "lost tracking"**: VO lost its features (open
  water, flat seafloor). Try, in order:
  - Pitch the depth camera down 20–30° in `model.sdf`, and update the static
    TF pitch in `bridge.sh` to match.
  - Fly closer to the pipe or structure.
  - Use depth ICP: `vo.sh -p Reg/Strategy:="'1'"`, or `'2'` for visual + ICP.

  rtabmap is run without auto-reset (`Odom/ResetCountdown 0`). A reset would
  jump its frame back to identity, which the EKF would see as a teleport.
- **GUIDED rejected / EKF never OK**: ArduSub needs an origin and ExtNav
  samples.
  - Confirm `EKF origin set` was logged and `sent` is non-zero.
  - Check params in the SITL console: `param show EK3_SRC1*`, `param show VISO*`.
  - Stale params: delete `./ardupilot_vision/eeprom.bin` to re-apply the overlay.
- **Yaw drifts against VO**: heading comes from the compass, VO position from
  the camera. Check that `COMPASS_DEC` matches the location (see above). As a
  fallback, `EK3_SRC1_YAW 6` (ExtNav yaw) with `vision_bridge.py --no-align`
  keeps the frames consistent by construction, but then "north" is the
  vehicle's initial heading rather than true north.
- **Tuning**: `VISO_POS_M_NSE`, `VISO_VEL_M_NSE`, `VISO_YAW_M_NSE` (EKF trust
  in the vision data), `VISO_DELAY_MS` (camera-to-pose latency).

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

## Click-to-go in RViz

`make bringup-vision` opens RViz with `vision_nav.rviz`:

- **Green**: Gazebo truth (path + arrow). **Red**: what ArduSub's EKF believes
  (path + arrow), which is what it steers by. Their gap is the VO error.
- **2D Goal Pose** (toolbar): click a spot and drag for heading. `goal_bridge.py`
  switches ArduSub to GUIDED, arms it, and flies there at `VISION_DEPTH`
  (default 1 m; e.g. `make bringup-vision VISION_DEPTH=2`).
- Grid = surface plane, 1 m cells. Frame `odom` is Gazebo's world: x = east,
  y = north, origin = spawn.
- From a terminal, with a chosen depth (z < 0 = depth):
  `ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 2, y: 3, z: -1.5}, orientation: {w: 1}}}"`

The Gazebo GUI still works for watching the vehicle, but it can't send goals.

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
| `src/vision_nav/goal_bridge.py` | RViz `/goal_pose` → GUIDED target; publishes EKF pose/path and truth path for RViz (MAVLink udp 14557) |
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

- **RViz / Gazebo GUI "could not connect to display" and x11-setup says
  `/tmp/.docker.xauth is a directory`**: a container was started (bare
  `docker compose up`) before the cookie file existed, and Docker created a
  directory there. Run `docker compose down && sudo rmdir /tmp/.docker.xauth`
  once. The compose file now refuses to start in that case rather than
  creating the directory.
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

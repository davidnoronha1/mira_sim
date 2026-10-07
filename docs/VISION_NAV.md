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

## Pieces

| File | Role |
|---|---|
| `docker/ardusub_vision.parm` | GPS off, `VISO_TYPE=1`, EKF3 XY pos/vel from ExtNav, Z from baro, yaw from compass |
| `ardupilot-sitl-vision` (compose) | SITL with the overlay above; own `./ardupilot_vision` data dir so params don't leak into the normal SITL |
| `src/vision_nav/bridge.sh` | ros_gz_bridge for camera + GT odometry + `/clock`, static TF `base_link → front_camera_depth_optical` |
| `src/vision_nav/vo.sh` | `rtabmap_odom rgbd_odometry`; extra args are passed through (e.g. `-p Reg/Strategy:="'1'"`) |
| `src/vision_nav/vision_bridge.py` | odometry → MAVLink `ODOMETRY`, sets EKF origin, aligns heading |
| `src/vision_nav/waypoints.py` | waits for EKF position, GUIDED, arm, flies a YAML waypoint list |
| `src/vision_nav/compare_odom.py` | prints VO error vs ground truth once per second |

The `bluerov2_heavy` model (bluerov2_gz fork) carries the
`OdometryPublisher` plugin and sets `gz_frame_id` on `front_camera_depth`.

## Frames

- Gazebo world is ENU, bodies are FLU. ArduPilot wants NED / FRD.
- `gt`: world ENU → NED directly (north = Gazebo +y, east = +x).
- `vo`: rtabmap's odom frame is FLU anchored at the start pose, so its FRD is
  treated as "NED with north = initial heading".
- For both, the first sample is rotated about the vertical so its heading
  matches ArduSub's AHRS yaw (logged as `Aligned ... rotate N deg`). For `gt`
  that number should be ~0 (or the magnetic declination); anything large
  means a frame bug.
- The EKF origin is the spawn point, so waypoints are metres from spawn.

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

- **No images / 0 Hz**: rendering fell back to software (`/tmp/gz-render-env.sh`
  says `ogre`). Use the `mira-sim-gpu` service (`MIRA_GPU=1`).
- **rtabmap "Did not receive data"**: check the image frame with
  `ros2 topic echo --once /vision/camera_info --field header`. It should be
  `front_camera_depth_optical`. If not, `gz_frame_id` wasn't picked up; change
  the child frame in `bridge.sh` to whatever the header says.
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
  the camera. If the EKF fights, try `EK3_SRC1_YAW 6` (ExtNav yaw), and run
  `vision_bridge.py --no-align`, since alignment to the AHRS is then circular.
- **Tuning**: `VISO_POS_M_NSE`, `VISO_VEL_M_NSE`, `VISO_YAW_M_NSE` (EKF trust
  in the vision data), `VISO_DELAY_MS` (camera-to-pose latency).

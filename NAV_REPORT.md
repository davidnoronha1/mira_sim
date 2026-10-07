# Vision Navigation Report

GPS-free navigation for the BlueROV2 Heavy in the Mira simulator. ArduSub gets its position from visual odometry on a simulated RGB-D camera. It flies to goals clicked in RViz, and avoids obstacles with ArduPilot's own object-avoidance planner, fed by the same camera.

Branch: `feat/vision-nav` in both `mira_sim` and the `bluerov2_gz` fork. For day-to-day usage and tuning, see [docs/VISION_NAV.md](docs/VISION_NAV.md). This report covers what was built, why, what broke along the way, and what was measured.

---

## 1. Goal

> Take a depth map from the Gazebo simulation, feed it to ArduPilot as a vision pose estimate, and use GUIDED mode to go to different waypoints in the environment.

It grew into:
- RViz click-to-go
- the SAUVC competition pool
- the world and the AUV drawn in RViz
- goals highlighted in Gazebo
- depth-camera obstacle avoidance running natively inside ArduSub

## 2. Final architecture

```
                     Gazebo Harmonic (sauvc_competition.world)
                     bluerov2_heavy: RGB-D camera, 25 deg nose-down
                                  |
                  ros_gz_bridge (bridge.sh) + static TFs
                                  |
        +-------------------------+--------------------------+
        |                                                    |
  /vision/image + /vision/depth                       /vision/depth
        |                                                    |
  RTAB-Map rgbd_odometry (vo.sh)                obstacle_distance.py
        |  /vision/vo_odom                        (72 sectors, +/-0.25 m band)
        |                                                    |
  vision_bridge.py                                 MAVLink OBSTACLE_DISTANCE
  (FLU->NED, start offset, heading align)                udp 14558
        |  MAVLink ODOMETRY, udp 14555                       |
        +----------------------> ArduSub SITL <--------------+
                                 (patched: OA planner + proximity)
                                 EKF3: XY = ExtNav, Z = baro, yaw = compass
                                 GUIDED + BendyRuler + face-along-path
                                           ^
                                           | SET_POSITION_TARGET_LOCAL_NED, udp 14557
                                 goal_bridge.py  <---  RViz "2D Goal Pose"
                                    |   \
                     Gazebo /marker      RViz: EKF pose/path, truth path,
                     (goal highlight)    TF odom->base_link
                                 scene_markers.py -> world + AUV meshes in RViz
```

Ground truth from Gazebo (`/vision/gt_odom`) **never reaches ArduSub** in the default mode. It is only drawn in RViz (the green ghost AUV and the green path) and used by `compare_odom.py` to measure drift.

## 3. What was built, in order

### 3.1 First pipeline: VO → ArduSub → GUIDED waypoints (`77007e4`)
- **`ardupilot-sitl-vision`** compose service plus `docker/ardusub_vision.parm`:
  - GPS off. Horizontal position and velocity come from ExtNav (`VISO_TYPE 1`), depth from the barometer, heading from the compass.
  - It has its own `./ardupilot_vision` data dir, so the saved parameters (`eeprom.bin`) never leak the GPS-off settings into the normal SITL.
- **`src/vision_nav/`:**
  - `bridge.sh` bridges the camera, the ground-truth odometry and `/clock`, and publishes the static TFs.
  - `vo.sh` runs RTAB-Map `rgbd_odometry`.
  - `vision_bridge.py` turns odometry into MAVLink `ODOMETRY`. It handles the ENU/FLU to NED/FRD conversion, sets the EKF origin with `SET_GPS_GLOBAL_ORIGIN`, and aligns the heading.
  - `waypoints.py` flies YAML lists in GUIDED.
  - `compare_odom.py` reports VO error against ground truth.
- **bluerov2_gz:** a ground-truth `OdometryPublisher` plugin, and `gz_frame_id` on the RGB-D camera.
- **Images:** rtabmap and pymavlink added to the Gazebo image.

### 3.2 Fixes found by testing end to end (`e7bd79a`, `81448b5`)
- **MAVProxy missing (broke plain `make sitl` on master too).** Commit `7676665` moved MAVProxy into `/opt/mavenv`, but compose overrode `PATH`, so the SITL container exited with `mavproxy.py: No such file or directory`.
- **ArduSub silently ignored the vision data.** The ardupilot_gazebo plugin sends `no_time_sync=true`, and SITL reacts by defaulting `AHRS_EKF_TYPE` to 10 (simulator ground truth, no EKF). EKF variances read exactly 0. The overlay now forces EKF3, which is safe because the model runs `lock_step`. Without this, every "test" would have passed without using vision at all.
- **Heading drifted away from the VO frame.** With no GPS, compass auto-declination only applies once the EKF has a location, which is after the heading has already initialised. The EKF yaw then swung 4.7° over a few minutes. `COMPASS_DEC` is now pinned for the sim location (`0.082045` rad), with `COMPASS_AUTODEC 0`.
- **Alignment before the heading settled.** The EKF reports yaw 0 briefly while initialising, so the bridge now waits for a heading that has held steady for 3 s.
- **Origin lost on reboot.** The bridge polls `GPS_GLOBAL_ORIGIN`, so a rebooted autopilot gets its origin again.

### 3.3 RViz click-to-go (`fb0661a`)
- **`goal_bridge.py`:** RViz "2D Goal Pose" becomes a GUIDED target, switching to GUIDED and arming automatically. It also publishes the EKF pose and path plus the truth path.
- **Layout:** a ready-made `vision_nav.rviz`, and `rviz2` added to the image.
- **Defaults:** `VISION_SOURCE=vo` became the default.
- **X11 hardening.** A bare `docker compose up` had created `/tmp/.docker.xauth` as a root-owned *directory*, which broke X for every later run. Compose now uses `create_host_path: false`, and `x11-setup.sh` detects the bad directory.

### 3.4 One-command bringup (`b958a0d`)
- **`make bringup-vision`:** one tmux session with a window per process. Each window waits for the topics it needs instead of using fixed sleeps.
- **Startup checks:** the image is checked for the vision tools, and plain `up -d` is used so config changes take effect.
- **X cookie:** moved to a user-owned `/tmp/.mira-sim.xauth`.

### 3.5 SAUVC, the scene in RViz, goals in Gazebo (`830cbde`)
- **World-anchored frames.** VO is offset by the map's known start pose (`--start-enu`), so ArduSub's local frame equals the Gazebo world frame. Goals and waypoints are plain pool coordinates.
- **`scene_markers.py`:** parses the world SDF (includes, nested models, primitives, meshes, colours) into RViz markers. It draws the AUV mesh twice: solid at the EKF pose and as a translucent green ghost at the true pose.
- **Gazebo goal highlight** via the `/marker` service, GUI-only so the simulated camera can't see it.

### 3.6 Competition map, camera, obstacle avoidance (`96944b1`, bluerov2_gz `871f1eb`)
- **`worlds/sauvc_competition.world`:**
  - `sauvc.world` is the Gazebo Classic competition layout and doesn't run on Harmonic. This new world combines its layout (gate, orange flare, drums with the golf ball, three flares) with `sauvc25.world`'s working Harmonic setup.
  - `VISION_MAP=sauvc` is now the competition map, `sauvc_quali` the qualification map and `tacc` the TACC world.
- **Camera pitch.**
  - Forward-facing at 1 m depth, the camera mostly saw open water, and VO drifted badly.
  - Pitched 35° down: about 270 inliers and roughly 5 mm jitter, against about 140 and 130–290 mm before.
  - Settled at **25°** so that **one camera serves both VO and avoidance**: the floor fills the lower ~2/3 of the image, and the top edge sits 2.2° above horizontal.
- **ArduSub patch (`docker/ardusub_object_avoidance.patch`, 6 files, +39/−2):**
  - Stock ArduSub cannot do depth-camera avoidance. The OA planner (BendyRuler/Dijkstra) and the proximity library exist in ArduPilot and are wired into Copter and Rover. ArduSub never creates the planner (there are no `OA_` params) and never initialises or updates proximity, so `PRX`/`AVOID_` settings do nothing.
  - The patch does Copter's wiring: `AC_WPNav_OA` instead of `AC_WPNav`, the `OA_` parameter group, and proximity init plus a 200 Hz update task.
  - It also makes GUIDED position targets without a yaw follow `WP_YAW_BEHAVIOR` instead of freezing the heading. With `WP_YAW_BEHAVIOR 1` the AUV faces along its OA-bent path, so the forward camera looks where it's going.
- **`obstacle_distance.py`:** a port of ArduPilot's `d4xx_to_mavlink.py` (the RealSense script).
  - It projects every depth pixel into the level body frame using the camera mount from TF.
  - It keeps the points within ±0.25 m of the AUV's height: the hull's half-height of 0.127 m plus a margin.
  - It sends the nearest point per sector, 72 sectors, as `OBSTACLE_DISTANCE`, and publishes the same data as a LaserScan for RViz.
- **BendyRuler tuned for a pool:**
  - `OA_MARGIN_MAX 0.5` (default 5 m)
  - `OA_BR_LOOKAHEAD 3` (default 15 m)
  - `OA_DB_BEAM_WIDTH 1.2` (one sector; the default 5° inflates thin posts)
  - `PRX_FILT 1`
- **Full-size contact hull** (0.457 × 0.575 × 0.254 m).
  - The original collision box is a 5 cm-thick plate tuned for neutral buoyancy, so the visible vehicle sailed through gate bars.
  - The hull is added as an **STL mesh** because graded buoyancy skips mesh collisions. Gazebo logs `Unsupported collision geometry for graded buoyancy[5]`, which confirms it adds contacts but no lift (a box would add about 53 kg).
  - All four worlds that use the model use graded buoyancy.
- **`goal_bridge` changes:**
  - It flies position-only targets and turns to the RViz arrow's heading on arrival.
  - It sends each target 3 times, then stops. Resending every second restarted ArduSub's path and made the motion stutter.
  - It publishes `odom → base_link` TF.
- **Session and RViz:**
  - RViz gets a depth-map panel, obstacle rays and a clearer legend. ArduSub's pose now reaches RViz at 30 Hz instead of 10.
  - `bringup-vision` gains an obstacles window and a host shell window, and clears orphaned processes that held MAVLink ports.
  - `bringdown` kills the vision session last, so it completes even when run from inside the session.
  - The default goal depth is 1.2 m, putting the hull top 0.37 m under the gate's top bar.

## 4. Docker repair (host, not in the repo)

containerd's image store under `/mnt/shared/DAVID/CONTAINERD` had records pointing at blob and layer data that no longer existed on disk. `docker images`, `docker system df`, pulls and builds all failed. It was repaired **live, without a reset**, keeping every image, container and volume that still had data:
- **Dead image records:** about 30 removed via the Docker API. Removing them deleted only names whose content was already gone.
- **Dangling content records:** 119 removed. These were blob records whose file was missing, including the common empty layer `4f4fb700…`. A root script deleted only records with no file behind them.
- **Dangling snapshot records:** 99 found by comparing containerd's `meta.db` against the overlayfs backend's database, using a `bbolt` CLI built in a container. Of these, 65 were removed by a root script that re-checked each one live before deleting it. 34 remain, pinned by old leases from interrupted pulls; they have no data behind them and block nothing.
- **Build cache:** about 26 GB, which also referenced broken layers, was cleared.

Images whose content was already gone had to be dropped: `dronesimulationo3de-*`, `minibude-*`, `test-sycl`, `alpine`. They need a re-pull or rebuild.

## 5. Measured results

All results are from the simulator, measured against Gazebo ground truth.

| Test | Result |
|---|---|
| TACC, 4 m square, ground truth as the vision input | 6/6 waypoints, ~2 cm final error |
| TACC, same square, **VO only** | 6/6 waypoints, ~13 cm final error, max VO drift 34 cm, 0 lost frames |
| TACC, RViz-style goal (2 m E, 3 m N) on VO | reached within 6 cm, heading 358° (asked 0°) |
| SAUVC quali, ~28 m goal across the pool, camera level | ended 1.7 m off (~5% drift) |
| SAUVC competition, camera level, waypoint tour | 1.9 m off by waypoint 3; tour stalled |
| Camera pitched down (VO quality at rest) | ~270 inliers, ~5 mm jitter (vs ~140 and 130–290 mm level) |
| Camera at 25° (shared with avoidance) | ~260 inliers |
| Obstacle projection, synthetic test (pole at 2.46 m, 9.1° right) | reported 2.46 m, 9.1°, floor ignored |
| Obstacle messages to ArduSub | ~34 Hz |
| Patched ArduSub parameters live | `OA_TYPE 1`, `OA_MARGIN_MAX 0.5`, `OA_BR_LOOKAHEAD 3`, `PRX1_TYPE 2`, `WP_YAW_BEHAVIOR 1` |
| Goal straight through the flare and gate (user test) | avoided the gate and flare rather than colliding |
| VO / camera rate | 30 Hz, ~26 ms latency, RTX PRO 6000 |

## 6. Known limitations

- **It won't go *through* the gate on its own.** BendyRuler is reactive and takes the shortest clear path. A goal beyond the gate goes around it. See next steps.
- **VO drift grows with distance.** It was about 5% in the open pool before the camera was pitched. Long legs need re-anchoring, for example on known landmarks.
- **Horizontal planning only.** Height is handled only by the ±0.25 m obstacle band and the goal depth. An obstacle edge just above or below the hull is treated as a wall.
- **Gazebo exited twice mid-session with no crash trace.** It looked like the window being closed, but that wasn't confirmed.
- **Not in the published images.** The GHCR images don't include rtabmap, rviz2, pymavlink or the ArduSub patch. Build locally with `docker compose build ardupilot-sitl mira-sim-gpu`; the SITL build takes 30–45 minutes.
- **Unpinned patch.** The ArduSub patch was written against master @ `30a9288`. `ardupilot.Dockerfile` clones master unpinned, so a future master may need the patch regenerated.

## 7. Next steps

1. **Pass through the gate.** Either:
   - insert a line-up point and a pass point in the gate opening, below the 0.7 m bar, in `goal_bridge` (uses the known map), or
   - use ArduPilot's `OA_TYPE 3` (Dijkstra with BendyRuler) with exclusion fences around the posts and flare, so ArduSub plans through the opening natively.
2. **Reduce long-range VO drift.** Try RTAB-Map `Reg/Strategy 2` (visual + depth ICP), loop closure, or landmark re-localisation.
3. **Upstream the ArduSub OA/proximity wiring.** It is a small, Copter-style change.
4. **Publish the vision-capable images from CI.**

## 8. How to run

```bash
docker compose build ardupilot-sitl mira-sim-gpu   # once
make bringup-vision                                # SAUVC competition pool
#   RViz "2D Goal Pose" to send the AUV somewhere
make waypoints                                     # fly the map's tour (from the tmux shell window)
make compare-odom                                  # VO drift vs truth
make bringdown
```

Options: `VISION_MAP=sauvc|sauvc_quali|tacc`, `VISION_DEPTH=1.2`, `HEADLESS=1`, `VISION_SOURCE=gt` (debugging only).

## 9. Commits

mira_sim `feat/vision-nav`:

| Commit | Summary |
|---|---|
| `77007e4` | GPS-free ArduSub from RGB-D visual odometry with GUIDED waypoints |
| `e7bd79a` | put the MAVProxy venv on the SITL container PATH |
| `81448b5` | force EKF3, pin declination, align heading once settled |
| `fb0661a` | click-to-go from RViz, VO as the default source |
| `b958a0d` | one-command bringup-vision tmux session |
| `830cbde` | SAUVC pool by default, world + AUV in RViz, goal shown in Gazebo |
| `96944b1` | ArduSub-native obstacle avoidance on the SAUVC competition map |

bluerov2_gz `feat/vision-nav` (fork):

| Commit | Summary |
|---|---|
| `01f422a` | ground-truth odometry publisher and optical frame for the RGB-D camera |
| `871f1eb` | full-size contact hull, depth camera pitched 25° down |

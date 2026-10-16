# Patches

Changes to third-party code, applied when the images are built.

| Patch | Applies to | Applied by | Why |
|---|---|---|---|
| `ardupilot/ardusub_object_avoidance.patch` | ArduPilot (ArduSub) | `docker/ardupilot.Dockerfile` | Enables the object-avoidance path planner (BendyRuler) and proximity in ArduSub GUIDED, and face-along-path yaw (`WP_YAW_BEHAVIOR`). Stock ArduSub ignores the `OA_*`/`PRX*` settings in `config/ardupilot/sim.parm`. Simulator only: the vehicle's Pixhawk runs stock firmware. |

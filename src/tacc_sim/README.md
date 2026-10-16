# tacc_sim

TACC competition world for Gazebo Harmonic, the counterpart of `sauvc_sim`.

- `worlds/tacc.world`: seafloor, pipeline and structure; the BlueROV2 Heavy
  comes from `bluerov2_gz`.
- `models/`: `pipe` (with ArUco marker), `structure`, `tacc_seafloor`.

The simulator containers mount this folder at `/workspace/tacc_sim`, with
`models/` and `worlds/` on `GZ_SIM_RESOURCE_PATH`. Run it with
`python3 bringup.py up --mode gazebo --map tacc`, `python3 bringup.py up --map tacc`, or the Mira stack with
`VISION_MAP=tacc`. The world's `<spherical_coordinates>` must match SITL's
`--custom-location` (docker-compose.yml).

To fix the seafloor orientation, change the `<pose>` in
`models/tacc_seafloor/model.sdf`.

Competition settings live in `config/ardusub.parm`. Global simulator ArduPilot settings live in the parent workspace’s `config/ardupilot/`.

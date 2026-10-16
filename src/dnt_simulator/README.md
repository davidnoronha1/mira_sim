# dnt_simulator

Gazebo to ROS plumbing for the simulator, as launch files. The containers run
them straight from the mounted source (`/workspace/dnt_simulator`), e.g.
`ros2 launch /workspace/dnt_simulator/launch/bridge.launch.py`; no build needed.

| Launch file | Used by | Bridges (config/) |
|---|---|---|
| `launch/bridge.launch.py` | Simulator (`python3 bringup.py up`); Mira connects separately | `bridge.yaml`: front RGB-D, bottom camera, IMU, ground truth, `/clock`; plus camera/IMU static TFs |
| `launch/competition_bridge.launch.py [competition:=sauvc]` | `python3 bringup.py up --map <map>` | `competition_bridge.yaml` (+ `sauvc_bridge.yaml`) |

The static TF mounts in `bridge.launch.py` must match `bluerov2_gz/models/bluerov2_heavy/model.sdf`.

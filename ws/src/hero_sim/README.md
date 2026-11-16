# hero_sim — HeRo in Gazebo Harmonic (ROS 2 Jazzy)

Simulates the [HeRo](../../../hero/README.md) swarm robot in the OBR rescue room (default) or a simple
test arena. Robot geometry and meshes come from `hero/hero_jazzy` (original HeRo dimensions:
64.6 mm track, 25 mm wheel radius). The sensors are ported from the ROS 1 `hero_description`.

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | input; capped at 0.2 m/s, 3 rad/s |
| `/odom` | `nav_msgs/Odometry` | 30 Hz, `odom` → `base_link` |
| `/laser` | `sensor_msgs/LaserScan` | 16 rays, 360°, 0.038–0.20 m, 20 Hz, **best-effort QoS** |
| `/imu` | `sensor_msgs/Imu` | 20 Hz, frame `base_link` |
| `/joint_states`, `/tf`, `/tf_static`, `/clock`, `/robot_description` | | sim time is on everywhere |

## 1. Build

A workspace colcon fica dentro deste repositório, em `ws/`.

```bash
cd /home/victor/obr_ros/ws
rosdep install --from-paths src -iy
colcon build --symlink-install
source install/setup.zsh
```

O build deve listar apenas `hero_jazzy` e `hero_sim`. Os pacotes ROS 1 (catkin) em `src/hero/` carregam
um `COLCON_IGNORE`, então colcon e rosdep os pulam.

O `--symlink-install` faz com que edições em `../area_resgate_obr.sdf` e nos arquivos de
`hero_sim/` (launch, urdf, config, worlds) valham imediatamente na próxima execução, sem rebuild.

## 2. Run

```bash
cd /home/victor/obr_ros/ws
source install/setup.zsh

ros2 launch hero_sim sim.launch.py                # OBR rescue room, Gazebo GUI + RViz
ros2 launch hero_sim sim.launch.py rviz:=false    # Gazebo only
ros2 launch hero_sim sim.launch.py gui:=false     # headless Gazebo server (RViz still opens)
ros2 launch hero_sim sim.launch.py x:=-0.65 y:=-0.325   # spawn on the entrance line tile, facing the room
```

Abra um segundo terminal para teleoperação:

```bash
cd /home/victor/obr_ros/ws
source install/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Worlds (`world:=`)

| Value | Scene |
|---|---|
| `obr_rescue` (default) | OBR 2026 rescue room (level 2), from `../area_resgate_obr.sdf` (i.e. `ws/src/area_resgate_obr.sdf`): a 0.94 m room with an entrance at the bottom of the west wall (silver tape) and an exit at the top of the east wall (black tape), line tiles outside both, green and red rescue-area barriers, two silver victims, and one black victim. |
| `hero_arena` | 1.5 m × 1.5 m walled test arena with four obstacles (`worlds/hero_arena.sdf`) |
| `/any/path/world.sdf` | any Gazebo Harmonic world |

The OBR world is not copied: CMake installs it from `../area_resgate_obr.sdf` as `worlds/obr_rescue.sdf`. With
`--symlink-install`, edits to the SDF file show up on the next launch without rebuilding. Override its location with
`colcon build --cmake-args -DOBR_RESCUE_WORLD=/path/to/file.sdf`.

Requirements for a custom world: it must load `gz-sim-sensors-system` (with `ogre2`) or `/laser` stays silent. It must
**not** load `gz-sim-imu-system`, because the robot model loads that itself and `/imu` would be published twice.

The OBR world has no ground plane, so a robot driven past the edge of the room or the tiles falls off.

## 3. Drive

Keep the teleop terminal focused. The driving keys are **not** WASD or the arrow keys:
```
   u    i    o        i / ,     forward / backward
   j    k    l        j / l     turn left / right on the spot
   m    ,    .        u o m .   curved moves        k = stop
```
`q`/`z`, `w`/`x` and `e`/`c` only change the speed limits: they print `currently: speed … turn …`
and do not move the robot. Driving keys print nothing. Each press sends one command, and the robot keeps
it until you press `k`. Teleop starts at 0.5 m/s and the robot caps at 0.2 m/s. Press `z` a few times for finer control.

You can also drive from the command line:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.5}}"
# Ctrl-C, then stop the robot (DiffDrive keeps the last command):
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 4. Inspect

```bash
ros2 topic echo /laser --qos-reliability best_effort --field ranges   # inf = nothing within 20 cm
ros2 topic hz /imu
ros2 run tf2_ros tf2_echo odom base_link
```

The laser's first ray points backwards (`angle_min = -π`). Rays step by 22.5° counter-clockwise, so index 8 points straight ahead.

## Files

| Path | Purpose |
|---|---|
| `launch/sim.launch.py` | Gazebo + spawn + robot_state_publisher + bridge + RViz |
| `urdf/hero_sim.urdf.xacro` | robot, DiffDrive, JointStatePublisher, gpu_lidar, IMU |
| `worlds/hero_arena.sdf` | simple test arena (edit to add or move obstacles) |
| `../../area_resgate_obr.sdf` | OBR rescue room, installed as `worlds/obr_rescue.sdf` |
| `config/bridge.yaml` | ROS ↔ Gazebo topic mapping |
| `rviz/hero.rviz` | RViz layout (fixed frame `odom`) |

## Troubleshooting

- **Black or glitchy Gazebo window**: check `glxinfo -B | grep renderer` (it should show D3D12). As a fallback, run
  `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch hero_sim sim.launch.py`. The laser is a `gpu_lidar`, so the Gazebo
  server needs working rendering even with `gui:=false`.
- **Empty `/laser` in `ros2 topic echo`**: add `--qos-reliability best_effort`.
- **Leftover Gazebo after Ctrl-C**: `pkill -9 -f "gz sim"`.
- Harmless warnings: `KDL does not support a root link with an inertia` and
  `gz_frame_id ... not defined in SDF`. Gazebo still applies the frame id.
- `CMake Error: The source ... does not match the source ... used to generate cache` after moving
  `hero_sim`: run `rm -rf ../build/hero_sim ../install/hero_sim` and build again.

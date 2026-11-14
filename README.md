# ObrRescue

OBR rescue-room worlds for Gazebo Harmonic and a HeRo robot simulation (`hero_sim`), running
in a ROS 2 Jazzy + Gazebo Harmonic Docker container with WSL2 GUI support.

## Folder layout

This repo lives in the `src/` folder of a colcon workspace, **next to the HeRo repo** (`hero_sim` uses
`hero/hero_jazzy` for the robot meshes and Gazebo GUI config):
```
<workspace>/src/
  ObrRescue/                 this repo (docker-compose.yml, Dockerfile, worlds, hero_sim/)
  hero/                      https://github.com/gmdeoliveira/hero, branch ros2-jazzy-migration
```
`docker-compose.yml` mounts the whole `src/` folder (the parent of this repo) as `~/ws/src` in the container.

Fresh setup on a new machine:
```bash
mkdir -p ~/OBR/ws/src && cd ~/OBR/ws/src
git clone https://github.com/victorbahh/ObrRescue.git
git clone -b ros2-jazzy-migration https://github.com/gmdeoliveira/hero.git
# The ROS 1 (catkin) packages in hero/ have to be skipped by colcon and rosdep:
for p in hero_ar_tracker hero_bringup hero_common hero_description hero_examples \
         hero_firmware hero_gazebo hero_genesis; do touch hero/$p/COLCON_IGNORE; done
```

## Docker usage

GUI apps (Gazebo, RViz2) display through WSLg. They are GPU-accelerated on the NVIDIA GPU through WSL's
D3D12 driver (`/dev/dxg`), so no NVIDIA container toolkit is needed.

From this repo's root:
```bash
docker compose up -d --build     # build + start (container: ros-jazzy)
docker compose exec ros bash     # open a shell (repeat for more terminals)
docker compose down              # stop
```

Inside the container:
```bash
cd ~/ws && rosdep install --from-paths src -iy && colcon build --symlink-install
source install/setup.bash
```

## HeRo simulation

The HeRo robot runs in the OBR rescue room (`world:=obr_rescue`, the default, from
[area_resgate_obr.sdf](area_resgate_obr.sdf)) or in a test arena (`world:=hero_arena`). It has a proximity laser,
an IMU and RViz. Details are in [hero_sim/README.md](hero_sim/README.md).
```bash
ros2 launch hero_sim sim.launch.py                       # after building as above
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # 2nd shell; press z to slow down
```

## Quick checks
```bash
glxinfo -B | grep renderer            # expect: D3D12 (NVIDIA GeForce RTX ...)
gz sim shapes.sdf                     # Gazebo GUI
rviz2
ros2 launch ros_gz_sim_demos diff_drive.launch.py
```

## Troubleshooting
- Renderer shows `llvmpipe`: check that `GALLIUM_DRIVER=d3d12` is set and that `/dev/dxg` exists on the host.
- Gazebo rendering glitches: try `gz sim --render-engine ogre shapes.sdf`,
  or software rendering with `LIBGL_ALWAYS_SOFTWARE=1`.
- `CMake Error: The source ... does not match the source ... used to generate cache` after moving
  `hero_sim`: run `rm -rf ~/ws/build/hero_sim ~/ws/install/hero_sim` and build again.

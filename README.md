# ObrRescue

OBR rescue-room worlds for Gazebo Harmonic e simulação do robô HeRo (`hero_sim`), rodando
localmente com ROS 2 Jazzy + Gazebo Harmonic (sem Docker).

## Estrutura da workspace

Tudo fica dentro da pasta `ws/` deste repositório:

```
obr_ros/                 este repositório
  ws/
    src/
      area_resgate_obr.sdf   mundo da sala de resgate OBR
      hero_sim/              pacote ROS 2 (este repositório)
      hero/                  https://github.com/gmdeoliveira/hero, branch ros2-jazzy-migration
    build/
    install/
    log/
```

O `.gitignore` já ignora `build/`, `install/` e `log/` em qualquer nível.

## Setup inicial

```bash
# Dentro deste repositório (/home/victor/obr_ros)
cd /home/victor/obr_ros

# Clonar o repositório HeRo dentro de ws/src/
git clone -b ros2-jazzy-migration https://github.com/gmdeoliveira/hero.git ws/src/hero

# Ignorar pacotes ROS 1 (catkin) do hero para o colcon e rosdep pular
for p in hero_ar_tracker hero_bringup hero_common hero_description hero_examples \
         hero_firmware hero_gazebo hero_genesis; do
  touch ws/src/hero/$p/COLCON_IGNORE
done
```

## Build

O ROS 2 Jazzy já está no seu `.zshrc` (`source /opt/ros/jazzy/setup.zsh`).

```bash
cd /home/victor/obr_ros/ws
rosdep install --from-paths src -iy
colcon build --symlink-install
source install/setup.zsh
```

O `--symlink-install` faz com que edições em `ws/src/area_resgate_obr.sdf` e nos arquivos de
`ws/src/hero_sim/` (launch, urdf, config, worlds) valham imediatamente na próxima execução,
sem precisar rebuildar.

## Rodar

```bash
cd /home/victor/obr_ros/ws
source install/setup.zsh

# Terminal 1 — simulação (Gazebo + RViz)
ros2 launch hero_sim sim.launch.py

# Terminal 2 — teleoperação
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Argumentos úteis do launch:
```
ros2 launch hero_sim sim.launch.py world:=hero_arena      # arena de teste
ros2 launch hero_sim sim.launch.py gui:=false             # headless (mantém RViz)
ros2 launch hero_sim sim.launch.py rviz:=false            # só Gazebo
ros2 launch hero_sim sim.launch.py x:=-0.65 y:=-0.325     # spawn na linha de entrada
```

## Verificações rápidas

```bash
source /opt/ros/jazzy/setup.zsh
gz sim shapes.sdf                 # Gazebo GUI abre?
rviz2                             # RViz abre?
ros2 pkg prefix hero_sim          # pacote construído?
ros2 pkg prefix hero_jazzy        # pacote hero_jazzy construído?
```

## Arquivos

| Caminho | Propósito |
|---|---|
| `ws/src/area_resgate_obr.sdf` | Mundo da sala de resgate OBR (instalado como `worlds/obr_rescue.sdf`) |
| `ws/src/hero_sim/launch/sim.launch.py` | Gazebo + spawn + robot_state_publisher + bridge + RViz |
| `ws/src/hero_sim/urdf/hero_sim.urdf.xacro` | Robô, DiffDrive, JointStatePublisher, gpu_lidar, IMU |
| `ws/src/hero_sim/worlds/hero_arena.sdf` | Arena de teste simples |
| `ws/src/hero_sim/config/bridge.yaml` | Mapeamento de tópicos ROS ↔ Gazebo |
| `ws/src/hero_sim/rviz/hero.rviz` | Layout do RViz (fixed frame `odom`) |

## Troubleshooting

- **Janela do Gazebo preta ou com falhas**: verifique `glxinfo -B | grep renderer` (deve mostrar D3D12).
  Fallback: `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch hero_sim sim.launch.py`.
- **`/laser` vazio**: use `--qos-reliability best_effort` no `ros2 topic echo`.
- **Gazebo sobrando após Ctrl-C**: `pkill -9 -f "gz sim"`.
- **Erro de cache do CMake após mover `hero_sim`**: rode `rm -rf ws/build/hero_sim ws/install/hero_sim` e build novamente.

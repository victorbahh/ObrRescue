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
ros2 launch hero_sim sim.launch.py mapping_lidar:=true     # laser panorâmico para mapeamento
```

## Ground truth e mapa para localização

Na máquina com ROS 2 Jazzy e Gazebo Harmonic, instale as dependências e reconstrua os
pacotes após atualizar os arquivos:

```bash
cd /home/victor/obr_ros/ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select hero_sim mcl_localization
source install/setup.bash
```

O caminho é um exemplo; ajuste-o para a máquina onde o repositório está instalado.
Em terminais adicionais, faça `source /opt/ros/jazzy/setup.bash` e, dentro de `ws`,
`source install/setup.bash`.

### Pose verdadeira

Inicie a simulação com `ros2 launch hero_sim sim.launch.py`. O Gazebo publica a pose
do modelo no mundo, encaminhada para `/ground_truth/pose` como
`geometry_msgs/msg/PoseStamped`, com tempo de simulação. Confira e registre:

```bash
ros2 topic echo /ground_truth/pose
ros2 bag record -o experimento_mcl /clock /ground_truth/pose /odom /laser /tf /tf_static
```

Encerre a gravação com Ctrl+C. Acrescente à gravação o tópico de estimativa que
seu MCL vier a publicar. `/odom` é a odometria das rodas, não o ground truth.
A pose publicada é a pose do modelo `hero`, e não uma transformação adicional em
`/tf`; o robô tem `base_link` como link principal. Confira o `header.frame_id` da
mensagem, que identifica o referencial do Gazebo, antes de fazer transformações.

### Criar e salvar o mapa

Para testar um mapeamento com maior cobertura, inicie ou reinicie a simulação com:

```bash
ros2 launch hero_sim sim.launch.py mapping_lidar:=true
```

Essa opção substitui temporariamente o laser de 70° por um laser simulado de 360°
com 360 amostras e o mesmo alcance e ruído. Ela não representa necessariamente o
sensor físico do HeRo. Para o experimento de localização, reinicie sem a opção
para voltar ao sensor original; o mapa salvo continua utilizável.

Com a simulação rodando, em outro terminal inicie o mapeamento:

```bash
ros2 launch mcl_localization mcl_launch.py
```

Em um terceiro terminal, mova o robô lentamente por toda a área com
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`. No teleop, reduza a
velocidade com `z`, evite colisões e giros bruscos e volte ao ponto inicial depois
de percorrer a sala. O `slam_toolbox` publica
`/map` (`nav_msgs/msg/OccupancyGrid`) e a transformação `map` -> `odom`. No RViz,
adicione um display **Map**, selecione `/map` e use `map` como Fixed Frame.
Para salvar o mapa em outro terminal:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/obr"
```

Isso cria `maps/obr.yaml` e `maps/obr.pgm`. Pare o mapeamento antes de usar o
mapa salvo como entrada do MCL. Para voltar a publicar o mapa salvo:

```bash
ros2 run nav2_map_server map_server --ros-args -p use_sim_time:=true -p yaml_filename:="$PWD/maps/obr.yaml"
```

Em outro terminal, ative o servidor de mapa:

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 topic echo /map --once --qos-durability transient_local
```

O mapa do SLAM é uma estimativa, não o mapa exato do SDF. Para investigar um mapa
ruim, adicione também um display **LaserScan** no RViz, com o tópico `/laser`:

- Leituras devem aparecer nas paredes e girar junto com o robô. Um anel muito
  próximo do centro pode indicar que o laser está detectando o próprio robô.
- Se não houver leituras ou mapa, confira `ros2 topic hz /laser`,
  `ros2 topic echo /clock --once` e os avisos do terminal do `slam_toolbox`.
- Se as paredes aparecerem duplicadas ou deformadas, compare a movimentação em
  `/odom` com `/ground_truth/pose`; são referenciais diferentes, então compare
  deslocamentos relativos, não coordenadas absolutas sem alinhamento.

O laser original cobre aproximadamente 70° e tem alcance de 2 m; com ele, gire o
robô para observar a área. As paredes finas e o ruído podem exigir ajustes nos
parâmetros de `ws/src/mcl_localization/config/slam.yaml`.

O referencial `map` do SLAM não é necessariamente o referencial do mundo Gazebo,
especialmente com spawn diferente de zero; correções do SLAM também podem alterar
a relação entre `map` e `odom`. Para calcular erro do MCL contra o ground truth,
primeiro alinhe `map` com o mundo e compare poses no mesmo referencial e instante.
O mapa salvo não publica por si só `map` -> `odom`: o MCL fornece essa
transformação e não deve ser executado junto com outro publicador dela.

### Localização no mapa salvo

O pacote contém uma primeira implementação de MCL: ela usa `/laser`, a
transformação `odom` -> `base_link` e o mapa salvo. Ela requer uma pose inicial
aproximada; não faz localização global automática. Pare o `slam_toolbox` e qualquer
outro servidor de mapa antes de iniciar, para não duplicar `/map` nem `map` -> `odom`.

Na máquina com ROS, reconstrua o pacote após atualizar os arquivos:

```bash
cd /home/victor/obr_ros/ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select mcl_localization
source install/setup.bash
```

Em um terminal inicie a simulação sem seu RViz padrão:

```bash
ros2 launch hero_sim sim.launch.py rviz:=false
```

Em outro terminal, após carregar os mesmos ambientes ROS e workspace:

```bash
cd /home/victor/obr_ros/ws
ros2 launch mcl_localization localization.launch.py map:="$PWD/maps/obr.yaml"
```

O launch inicia o servidor de mapa, o MCL e um RViz configurado. No RViz,
selecione **2D Pose Estimate** e arraste a seta no ponto e na direção aproximados
do robô no mapa. Mova o robô lentamente; `/mcl/pose` mostra a estimativa e
`/mcl/particles` mostra as partículas. Use a ferramenta novamente se precisar
reinicializar. A dispersão inicial é limitada a 10 cm e 0,5 rad para esta arena;
os parâmetros `max_initial_std_xy` e `max_initial_std_yaw` do nó ajustam esses limites.

### Visualizar o ground truth

O tópico original `/ground_truth/pose` está nas coordenadas do mundo do Gazebo.
O nó de MCL republica a mesma pose em `/ground_truth/rviz`, no referencial
`ground_truth_world`. Para visualizá-la sem alinhamento, altere temporariamente o
**Fixed Frame** do RViz para `ground_truth_world`; mapa e robô estimado não terão
transformação nesse modo.

Para desenhar a pose verdadeira sobre o mapa, é necessário conhecer a transformação
fixa entre mundo e mapa. O launch permite informá-la explicitamente:

```bash
ros2 launch mcl_localization localization.launch.py map:="$PWD/maps/obr.yaml" \
  ground_truth_alignment:=true world_x:=0.0 world_y:=0.0 world_yaw:=0.0
```

Substitua `world_x`, `world_y` e `world_yaw` pela pose da origem do mundo do
Gazebo expressa em `map`, em metros e radianos. Zero só é correto quando os dois
referenciais realmente coincidem; a origem do YAML do mapa não é essa transformação.
Para determinar o alinhamento, compare uma pose conhecida do robô em ambos os
referenciais: o ângulo é `yaw_map - yaw_mundo`, e a translação é a posição no
mapa menos a posição no mundo girada por esse ângulo. O mapa construído por SLAM
pode ter deformações, portanto esse alinhamento pode ser apenas aproximado.

Se não aparecer a estimativa, verifique se `/map`, `/laser` e `/clock` estão
ativos, se o simulador está rodando e se a pose inicial foi enviada. Consulte os
avisos do terminal do MCL para problemas de TF ou ausência de leituras válidas.

### Navegação pelo RViz

Essa demonstração usa `area_resgate_obr.sdf` e um mapa estático gerado a partir
das caixas de colisão do mundo. As vítimas não são incluídas como paredes: o
Nav2 usa o laser para atualizar os obstáculos e replanejar. O mapa anterior do
SLAM não é modificado.

Na máquina com ROS, pare as outras simulações e nós de mapeamento/localização:

```bash
cd /home/victor/obr_ros/ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select hero_sim mcl_localization
source install/setup.bash
ros2 launch mcl_localization rescue.launch.py
```

O robô nasce em `(-0.65, -0.325)`, voltado para `+x`. No RViz, use **2D Pose
Estimate** para marcar essa pose e direção no mapa; aguarde a estimativa do MCL.
Depois, use **2D Goal Pose** para indicar o destino e sua orientação. Nenhum
destino é enviado automaticamente. O laser simulado de 360° publica possíveis
alvos em `/victims/markers` e a trajetória aparece no RViz.
O desvio usa **todas** as leituras do laser, não apenas os objetos classificados.
O Collision Monitor bloqueia os comandos da missão quando observa algo muito
próximo ou perde o laser; comandos publicados diretamente em `/cmd_vel` não passam
por ele. Encerre o launch com Ctrl+C para interromper a missão.

Os marcadores amarelos indicam até três **possíveis** vítimas por tamanho,
distância das paredes e quantidade de observações recentes. Isso não é uma
probabilidade calibrada. Um laser 2D não identifica cor nem diferencia vítimas vivas e mortas;
também pode haver falsos positivos, perdas por oclusão e objetos não observados
no trajeto. Os marcadores expiram quando deixam de ser observados; não há garantia
de que todas as vítimas serão vistas. Esse fluxo é experimental:
acompanhe o primeiro teste e não o utilize em um robô real sem validação adicional.

Se o planejamento falhar, verifique se existe apenas um publicador de `/map` com
`ros2 topic info /map -v`. Também confira as dimensões do mapa recebido com
`ros2 topic echo /map --once --field info --qos-durability transient_local`: o
mapa gerado atualmente para esta arena tem 170 por 116 células. Feche outras
simulações, servidores de mapa e nós de SLAM antes de iniciar este launch.

Para diagnosticar paradas, acompanhe os avisos do `controller_server` e do
`collision_monitor`. O robô pode recuar lentamente e tentar limpar os costmaps
após uma falha; o monitor ainda para a navegação quando o laser falha ou detecta
obstáculos na zona correspondente ao movimento. Não desative essa proteção para
contornar bloqueios.

Para a simulação, o raio de navegação é 0,045 m e a zona de parada dianteira é
0,065 m; a velocidade máxima é 0,04 m/s. Essas margens permitem aproximação maior,
mas o laser só mede a partir de 0,05 m e tem ruído: colisões ainda são possíveis.
Esses valores não são adequados para garantir segurança em um robô real.

No RViz, **Global Costmap** mostra os obstáculos usados pelo planejador e suas
margens de inflação. **Local Costmap** pode ser ativado para inspecionar o
controlador. Se uma vítima aparece no display **Laser** mas não no costmap,
verifique o TF, os avisos do Nav2 e os filtros de `obstacle_layer.scan` em
`ws/src/mcl_localization/config/navigation.yaml`. Os marcadores amarelos de
possíveis vítimas são independentes da marcação de obstáculos do Nav2.

### Ajustar o laser

O sensor é definido em `ws/src/hero_sim/urdf/hero_sim.urdf.xacro`, na seção
`hero_laser`. O launch `rescue.launch.py` usa `mapping_lidar:=true`, portanto
edite o bloco `<xacro:if value="$(arg mapping_lidar)">` dentro de `<horizontal>`:

- `<samples>`: número de feixes, atualmente 360.
- `<min_angle>` e `<max_angle>`: limites angulares em radianos. O campo de visão
  é a diferença entre eles. O atual vai de `-pi` a `pi - pi/180`.
- `<range><min>` e `<range><max>`: distâncias mínima e máxima em metros,
  atualmente 0,05 m e 2,0 m.
- `<range><resolution>`: resolução de distância em metros, atualmente 0,001 m.
- `<noise><stddev>`: desvio padrão do ruído gaussiano em metros, atualmente 0,01 m.
- `<update_rate>`: frequência de publicação em Hz, atualmente 100 Hz.

Os parâmetros de alcance, ruído e frequência são comuns aos dois modos do laser.
O bloco `<xacro:unless ...>` configura o outro modo, que não é usado por essa
missão. Ao alterar o número de feixes panorâmicos, ajuste também o ângulo máximo
se quiser manter o intervalo uniforme sem duplicar o primeiro e o último feixe.

Após editar, reconstrua `hero_sim` e reinicie a simulação. Confira o que está
efetivamente sendo publicado:

```bash
colcon build --symlink-install --packages-select hero_sim
source install/setup.bash
ros2 topic echo /laser --once --qos-reliability best_effort
```

No `LaserScan`, compare `angle_min`, `angle_max`, `angle_increment`, `range_min`,
`range_max` e a quantidade de valores em `ranges`. O MCL usa no máximo 40 feixes
por scan, configuráveis pelo parâmetro `max_beams` do nó; isso é diferente do
número de feixes produzidos pelo sensor. Os costmaps têm limites próprios
`obstacle_max_range` e `raytrace_max_range` em `navigation.yaml`; confira-os ao
alterar o alcance. Menos feixes ou alcance podem fazer o detector e o monitor de
colisão deixarem de observar uma vítima, especialmente durante a ré.

Se a geometria do SDF mudar, regenere o mapa antes de reconstruir o pacote:

```bash
PYTHONPATH=src/mcl_localization python3 -m mcl_localization.world_map \
  src/area_resgate_obr.sdf src/mcl_localization/maps/obr_world
colcon build --symlink-install --packages-select mcl_localization
```

O gerador atual considera piso e obstáculos de geometria `box` e poses planas;
novas geometrias, rampas ou poses relativas exigem adaptação do gerador.

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

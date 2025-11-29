---
sidebar_position: 2
---

# Instalação e Execução - 1ª Geração

## Criação do Workspace

```bash
# Criar diretório do workspace
mkdir -p ~/robodc_ws/src
cd ~/robodc_ws/src

# Inicializar workspace
catkin_init_workspace
```

## Clonar Repositórios

```bash
# Clonar todos os pacotes do RobôDC
cd ~/robodc_ws/src

# Pacote principal
git clone https://github.com/robodc/robodc_bringup.git

# Description e URDF
git clone https://github.com/robodc/robodc_description.git

# Controle
git clone https://github.com/robodc/robodc_control.git

# Sensores
git clone https://github.com/robodc/robodc_sensors.git

# Navegação
git clone https://github.com/robodc/robodc_navigation.git

# Mensagens customizadas
git clone https://github.com/robodc/robodc_msgs.git
```

:::tip Alternativa
Para clonar apenas o essencial para começar:
```bash
git clone --recursive https://github.com/robodc/robodc_1gen.git
```
:::

## Instalar Dependências

```bash
cd ~/robodc_ws

# Instalar dependências com rosdep
rosdep install --from-paths src --ignore-src -r -y
```

## Compilação

```bash
# Compilar com catkin_make
cd ~/robodc_ws
catkin_make

# OU compilar com catkin build (recomendado)
catkin build

# Source do workspace
source devel/setup.bash
```

:::note Adicionar ao bashrc
Para não precisar fazer source toda vez:
```bash
echo "source ~/robodc_ws/devel/setup.bash" >> ~/.bashrc
```
:::

## Verificar Compilação

```bash
# Listar pacotes instalados
rospack list | grep robodc

# Verificar variável ROS_PACKAGE_PATH
echo $ROS_PACKAGE_PATH
# Deve incluir: /home/seu_usuario/robodc_ws/src
```

## Execução - Robô Real

### 1. Iniciar ROS Core
Em um terminal:
```bash
roscore
```

### 2. Lançar Sistema Completo do Robô
Em outro terminal:
```bash
roslaunch robodc_bringup robot.launch
```

Este comando inicia:
- ✅ Robot State Publisher
- ✅ Todos os sensores (LIDAR, câmera, IMU)
- ✅ Base controller
- ✅ Robot Localization (EKF)

### 3. Lançar Navegação
Em outro terminal:
```bash
# Carregar mapa existente
roslaunch robodc_bringup navigation.launch map_file:=/path/to/your/map.yaml

# OU criar novo mapa com SLAM
roslaunch robodc_bringup slam.launch
```

### 4. Visualizar no RViz
```bash
roslaunch robodc_bringup rviz.launch
```

## Execução - Simulação (Gazebo)

:::info Simulação
A simulação é útil para desenvolver e testar sem o robô físico.
:::

```bash
# Lançar simulação completa
roslaunch robodc_bringup simulation.launch

# Lançar apenas Gazebo
roslaunch robodc_description gazebo.launch

# Lançar navegação na simulação
roslaunch robodc_navigation navigation_sim.launch
```

## Comandos de Teste

### Verificar Tópicos Ativos
```bash
rostopic list
```

Você deve ver:
```
/cmd_vel
/odom
/scan
/imu/data
/camera/image_raw
/tf
/tf_static
/move_base/goal
/map
...
```

### Verificar Frequência dos Tópicos
```bash
# LIDAR (deve ser ~5-10 Hz)
rostopic hz /scan

# Odometria (deve ser ~50 Hz)
rostopic hz /odom

# IMU (deve ser ~100 Hz)
rostopic hz /imu/data
```

### Publicar Velocidade Manualmente
```bash
# Mover para frente
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10

# Girar no lugar (Ctrl+C para parar)
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.3" -r 10
```

### Visualizar Transformadas (TF)
```bash
# Ver árvore de transformadas
rosrun tf view_frames

# Monitorar TF em tempo real
rosrun tf tf_echo map base_link
```

## Criar um Mapa (SLAM)

```bash
# Terminal 1: Lançar robô
roslaunch robodc_bringup robot.launch

# Terminal 2: Lançar SLAM
roslaunch robodc_bringup slam.launch

# Terminal 3: Controlar robô
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Terminal 4: Visualizar
rviz -d $(rospack find robodc_bringup)/rviz/slam.rviz
```

Dirija o robô pelo ambiente até mapear tudo. Quando terminar:

```bash
# Salvar mapa
rosrun map_server map_saver -f ~/robodc_ws/maps/meu_mapa
```

## Navegar Autonomamente

```bash
# Terminal 1: Robô + navegação
roslaunch robodc_bringup robot.launch
roslaunch robodc_bringup navigation.launch map_file:=~/robodc_ws/maps/meu_mapa.yaml

# Terminal 2: RViz
rviz -d $(rospack find robodc_bringup)/rviz/navigation.rviz
```

No RViz:
1. Use **2D Pose Estimate** para definir posição inicial
2. Use **2D Nav Goal** para enviar objetivo de navegação

## Troubleshooting

### Erro: "Unable to sync with device"
**Problema**: LIDAR não conecta
```bash
# Verificar porta USB
ls -l /dev/ttyUSB*

# Verificar permissões
sudo chmod 666 /dev/ttyUSB0

# Verificar se regra udev está ativa
ls -l /dev/rplidar
```

### Erro: "No transform from base_link to laser_frame"
**Problema**: TF não está sendo publicado
```bash
# Verificar robot_state_publisher
rosnode info /robot_state_publisher

# Verificar URDF
rosrun urdf_check urdf $(rospack find robodc_description)/urdf/robodc.urdf.xacro
```

### Erro: Camera não abre
```bash
# Listar dispositivos de vídeo
ls -l /dev/video*

# Testar câmera diretamente
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --all
```

### Robô não se move
```bash
# Verificar se cmd_vel está sendo recebido
rostopic echo /cmd_vel

# Verificar base_controller
rosnode info /base_controller

# Testar motores diretamente
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" -r 10
```

## Desligamento Seguro

```bash
# Parar todos os nós
Ctrl+C em todos os terminais

# OU usar rosnode
rosnode kill -a

# Desligar roscore
killall -9 roscore rosmaster

# Desligar Raspberry Pi (se aplicável)
sudo shutdown -h now
```

---
sidebar_position: 2
---

# Repositório vivaldini/ROBO_DC

## Informações do Repositório

- **Nome**: ROBO_DC
- **Owner**: vivaldini
- **URL**: https://github.com/vivaldini/ROBO_DC
- **Tecnologia Principal**: ROS 1 Noetic
- **Licença**: MIT License (2023, Robson Rogério Dutra Pereira)

## Objetivo

Repositório principal contendo os **pacotes ROS 1** para operação do robô móvel de 1ª geração do Departamento de Computação da UFSCar. Implementa controle de baixo nível, odometria, comunicação serial e simulação.

## Estrutura do Repositório

```
ROBO_DC/
├── mobile_rob_dev/              # Pacote principal do robô
│   ├── src/
│   │   └── mobile_rob_dev/
│   │       ├── robotMain.cpp    # Main loop (40 Hz)
│   │       ├── robotSystem.cpp  # Sistema do robô
│   │       └── robot.cpp        # Classe Robot
│   ├── include/
│   │   └── mobile_rob_dev/
│   │       ├── robotSystem.h
│   │       ├── robot.h
│   │       └── definitions.h
│   ├── scripts/
│   │   └── send_goal.py         # Enviar objetivos via action
│   ├── bash/
│   │   └── bag_datehour.sh      # Gravar rosbags
│   ├── launch/                  # Launch files
│   ├── CMakeLists.txt
│   └── package.xml
│
├── mobile_rob_dev_sim/          # Simulação no Gazebo
│   ├── config/                  # Parâmetros de simulação
│   ├── launch/                  # Launch files Gazebo
│   ├── meshes/                  # Modelos 3D
│   ├── urdf/                    # Descrição URDF
│   └── CMakeLists.txt
│
├── envrobotz/                   # Ambiente e configurações
│   ├── CMakeLists.txt
│   └── package.xml
│
├── api/                         # API Flask (legado)
│   ├── src/
│   │   └── controllers/
│   │       └── ros_controller.py
│   ├── app.py
│   └── README.md
│
├── bashrc.txt                   # Configuração do ROS
└── README.md
```

## Pacote: mobile_rob_dev

### Responsabilidades

1. **Comunicação Serial** (`robotSystem.cpp`)
   - Leitura de dados do Arduino via porta serial (`/dev/ttyACM0`, 115200 baud)
   - Recepção de posição (x, y, alpha) do hardware
   - Envio de comandos de velocidade

2. **Odometria** (`robot.cpp`)
   - Cálculo de odometria diferencial
   - Publicação de `/odom` (nav_msgs/Odometry)
   - Publicação de `/pose2d` (geometry_msgs/Pose2D)
   - Broadcast de TF (`odom` → `base_link`)

3. **Controle de Velocidade** (`cmdVelCallback`)
   - Subscrição de `/robot/cmd_vel` (geometry_msgs/Twist)
   - Conversão para comandos de motor

4. **Parâmetros do ROS** (`loadSettings`)
   - `wheel_radius`: Raio da roda
   - `robotRadius`: Raio do robô (distância entre rodas / 2)

### Nós Principais

```cpp
// Nó: mobile_rob_dev_node
ros::init(argc, argv, "mobile_rob_dev");

// Publishers
odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1);
pose_pub = n.advertise<geometry_msgs::Pose2D>("/pose2d", 1);

// Subscribers
cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/robot/cmd_vel", 1, &System::cmdVelCallback, this);
msg_sub = n.subscribe<std_msgs::String>("/robot/message", 1, &System::messageCallback, this);

// Loop rate: 40 Hz
ros::Rate loop_rate(40);
```

### Compilação

```bash
# Criar workspace
mkdir -p ~/laris_wksp/src
cd ~/laris_wksp/src

# Clonar repositório
git clone https://github.com/vivaldini/ROBO_DC.git

# Instalar dependências
cd ~/laris_wksp
rosdep install --from-paths src --ignore-src -r -y

# Compilar
catkin build -DCMAKE_BUILD_TYPE=Release
# OU
catkin_make -DCMAKE_BUILD_TYPE=Release

# Source
source devel/setup.bash
```

## Pacote: mobile_rob_dev_sim

### Responsabilidades

- Simulação do robô no **Gazebo 11**
- Modelos URDF/Xacro do robô
- Meshes 3D para visualização
- Launch files para simulação

### Uso

```bash
# Lançar simulação
roslaunch mobile_rob_dev_sim gazebo.launch

# Lançar navegação simulada
roslaunch mobile_rob_dev_sim navigation_sim.launch
```

## API Flask (Legado - api/)

:::warning Código Legado
A API Flask dentro deste repositório (`api/`) foi **substituída** pela API moderna em **Hugo-Souza/RoboDC_api**. A nova API tem melhor arquitetura, documentação Swagger e mais funcionalidades.
:::

### Funcionalidades (Legado)

- Enviar robô para locais pré-definidos
- Listar locais disponíveis
- Verificar status da navegação
- Cancelar objetivos

**Locais Cadastrados** (coordenadas em metros):
```python
available_locals = {
    "LE-1": (-37.99, -5.45, 1.0, 0.0),
    "LE-2": (-30.15, -5.03, 1.0, 0.0),
    "LE-3": (-22.68, -4.45, 1.0, 0.0),
    "LE-4": (-15.36, -4.11, 1.0, 0.0),
    "Suporte": (-11.30, -3.92, 1.0, 0.0),
    "PPG-CC4": (-2.54, -3.12, 1.0, 0.0),
    "Maker": (7.46, -2.39, 1.0, 0.0),
    "LE-5": (9.75, -2.36, 1.0, 0.0),
    "Auditorio": (15.37, -1.86, 1.0, 0.0),
    "Banheiros": (-38.74, -10.59, 1.0, 0.0),
    "Copa": (-38.43, -16.47, 1.0, 0.0),
    "Lig": (-38.01, -22.61, 1.0, 0.0),
    "Reunioes": (-15.52, -23.80, 1.0, 0.0),
    "Chefia": (-12.49, -23.54, 1.0, 0.0),
    "Graduacao": (-18.67, -24.17, 1.0, 0.0),
    "Recepcao": (-12.49, -23.54, 1.0, 0.0),
    "Home": (-1.65, -21.18, 1.0, 0.0)
}
```

## Dependências

### Pacotes ROS Necessários

```bash
sudo apt-get install \
    ros-noetic-amcl \
    ros-noetic-move-base \
    ros-noetic-navigation \
    ros-noetic-slam-gmapping \
    ros-noetic-rplidar-ros \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-teleop-twist-joy \
    ros-noetic-urg-node \
    ros-noetic-imu-filter-madgwick \
    ros-noetic-robot-localization \
    ros-noetic-teb-local-planner \
    ros-noetic-rtabmap-ros
```

### Dependências Git

```bash
cd ~/laris_wksp/src
git clone https://github.com/rrdpereira/envrobotz.git
git clone https://github.com/rrdpereira/pc2l.git
```

## Configuração Bash

```bash
# Adicionar ao ~/.bashrc
source /opt/ros/noetic/setup.bash
source ~/laris_wksp/devel/setup.bash

export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export ROS_WORKSPACES=~/laris_wksp

source /usr/share/gazebo/setup.sh
```

## Scripts Úteis

### Gravar Rosbags

```bash
# bash/bag_datehour.sh
mkdir -p ~/z_bags/$(date +%Y_%m_%d__%H_%M_%S) && cd $_
rosbag record /cmd_vel /odom /scan /tf /tf_static /map /amcl_pose -o mob_rob
```

### Enviar Objetivo (Python)

```python
# scripts/send_goal.py
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.pose.position.x = -37.99
goal.target_pose.pose.position.y = -5.45
goal.target_pose.pose.orientation.w = 1.0

client.send_goal(goal)
client.wait_for_result()
```

## Status do Repositório

- **Estado**: ✅ Ativo
- **Branch Principal**: `main`
- **ROS**: ROS 1 Noetic
- **OS**: Ubuntu 20.04 LTS
- **Última Atualização**: 2023-2024

## Equipe LARIS-UFSCar

- **Prof. Roberto Santos Inoue** - rsinoue@ufscar.br
- **Prof. Vivaldini**
- **Dr. Robson Rogério Dutra Pereira**
- **João Carlos Tonon Campi** (Graduação)
- **Leandro José Evilásio Campos** (Mestrado)
- **José Ceron Neto** (Mestrado)

## Links Relacionados

- 🔗 [Repositório no GitHub](https://github.com/vivaldini/ROBO_DC)
- 📖 [README Original](https://github.com/vivaldini/ROBO_DC/blob/main/README.md)
- 🏛️ [Site do DC-UFSCar](https://site.dc.ufscar.br/)
- 🤖 [LARIS - Laboratory of Autonomous Robots and Intelligent Systems](https://site.dc.ufscar.br/laris/)

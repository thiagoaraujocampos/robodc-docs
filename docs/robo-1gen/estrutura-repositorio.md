---
sidebar_position: 3
---

# Estrutura dos Repositórios

## Visão Geral

O repositório **vivaldini/ROBO_DC** contém os pacotes ROS 1 para operação do robô de 1ª geração. A estrutura segue o padrão de workspace ROS 1 com Catkin.

**Repositório**: https://github.com/vivaldini/ROBO_DC

## Estrutura Real do Repositório

```
ROBO_DC/
├── mobile_rob_dev/              # Pacote principal do robô
│   ├── src/
│   │   └── mobile_rob_dev/
│   │       ├── robotMain.cpp    # Main loop (40 Hz)
│   │       ├── robotSystem.cpp  # Comunicação serial, odometria
│   │       └── robot.cpp        # Classe Robot
│   ├── include/
│   │   └── mobile_rob_dev/
│   │       ├── robotSystem.h
│   │       ├── robot.h
│   │       └── definitions.h
│   ├── scripts/
│   │   └── send_goal.py         # Enviar objetivos via actionlib
│   ├── bash/
│   │   └── bag_datehour.sh      # Script para gravar rosbags
│   ├── launch/                  # Launch files (a criar)
│   ├── CMakeLists.txt
│   └── package.xml
│
├── mobile_rob_dev_sim/          # Simulação no Gazebo
│   ├── config/                  # Parâmetros de simulação
│   ├── launch/                  # Launch files Gazebo
│   ├── meshes/                  # Modelos 3D (.stl, .dae)
│   ├── urdf/                    # Descrição URDF do robô
│   ├── worlds/                  # Mundos Gazebo (.world)
│   ├── CMakeLists.txt
│   └── package.xml
│
├── envrobotz/                   # Ambiente e configurações
│   ├── CMakeLists.txt
│   └── package.xml
│
├── api/                         # API Flask (LEGADA - não usar)
│   ├── src/
│   │   └── controllers/
│   │       ├── ros_controller.py
│   │       └── metadata_controller.py
│   ├── app.py
│   ├── config.py
│   └── README.md
│
├── bashrc.txt                   # Configurações do ROS (source, ROS_MASTER_URI)
└── README.md
```

:::warning API Legada
A pasta `api/` contém código **legado** e não deve ser usada. A API moderna está no repositório separado **Hugo-Souza/RoboDC_api** (Flask-RESTX v1.2.3).
:::

## Pacote Principal: mobile_rob_dev

### Responsabilidades

1. **Comunicação Serial** com Arduino/Raspberry Pi Pico
2. **Cálculo de Odometria** diferencial
3. **Controle de Velocidade** (recebe `/robot/cmd_vel`, envia comandos aos motores)
4. **Publicação de TF** (`odom` → `base_link`)
5. **Publicação de tópicos** `/odom` e `/pose2d`

### Arquivos Principais

#### `src/mobile_rob_dev/robotMain.cpp`
```cpp
// Main loop do nó ROS
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile_rob_dev");
    ros::NodeHandle n;
    
    System system(&n);
    ros::Rate loop_rate(40);  // 40 Hz
    
    while (ros::ok()) {
        system.step();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
```

**Função**: Inicializa o nó ROS `mobile_rob_dev` e executa loop principal a 40 Hz.

#### `src/mobile_rob_dev/robotSystem.cpp`
```cpp
// Principais funções:
void System::setupSerial()      // Configura porta serial (/dev/ttyACM0, 115200)
void System::readSerial()       // Lê posição (x, y, theta) do Arduino/Pico
void System::writeSerial()      // Envia velocidades (vl, vr) para Arduino/Pico
void System::updateOdometry()   // Calcula e publica odometria
void System::cmdVelCallback()   // Recebe /robot/cmd_vel e converte para vl, vr
void System::loadSettings()     // Carrega parâmetros (wheel_radius, wheel_base)
```

**Protocolo Serial**:
- **Recebe**: `x,y,theta\n` (posição do robô)
- **Envia**: `vl,vr\n` (velocidades das rodas esquerda/direita)

#### `src/mobile_rob_dev/robot.cpp`
```cpp
// Classe Robot
void Robot::updatePose(float x, float y, float theta)  // Atualiza pose
void Robot::updateOdometry()                            // Calcula odometria diferencial
void Robot::computeVelocities(float vx, float wz)      // Converte (vx, wz) → (vl, vr)
```

**Cinemática Diferencial**:
```
vl = vx - (wz * wheel_base / 2)
vr = vx + (wz * wheel_base / 2)
```

### Parâmetros ROS (a ser definidos em launch file ou YAML)

```yaml
mobile_rob_dev:
  wheel_radius: 0.05       # Raio da roda (metros)
  wheel_base: 0.30         # Distância entre rodas (metros)
  max_linear_velocity: 0.5 # m/s
  max_angular_velocity: 1.0 # rad/s
  serial_port: /dev/ttyACM0
  serial_baud: 115200
  publish_rate: 40         # Hz
```

### Tópicos ROS

#### Publicados
- `/odom` (nav_msgs/Odometry): Odometria do robô
- `/pose2d` (geometry_msgs/Pose2D): Pose 2D simplificada
- `/tf`: Transformada `odom` → `base_link`

#### Subscritos
- `/robot/cmd_vel` (geometry_msgs/Twist): Comandos de velocidade
- `/robot/message` (std_msgs/String): Mensagens (opcional)

### Compilação

```bash
cd ~/laris_wksp
catkin build mobile_rob_dev -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### Execução

```bash
# Iniciar roscore
roscore &

# Iniciar nó mobile_rob_dev
rosrun mobile_rob_dev mobile_rob_dev_node

# Enviar comando de velocidade
rostopic pub /robot/cmd_vel geometry_msgs/Twist "linear: {x: 0.2}" -r 10
```

## Pacote: mobile_rob_dev_sim

### Responsabilidades

- Simulação do robô no **Gazebo 11**
- Modelos URDF/Xacro do robô
- Meshes 3D para visualização
- Launch files para simulação

### Estrutura

```
mobile_rob_dev_sim/
├── config/              # Parâmetros de simulação
├── launch/
│   ├── gazebo.launch    # Lançar apenas Gazebo
│   └── navigation_sim.launch  # Navegação na simulação
├── meshes/              # Modelos 3D (.dae, .stl)
├── urdf/
│   └── robodc.urdf.xacro  # Descrição URDF do robô
└── worlds/              # Mundos Gazebo (.world)
```

### Uso

```bash
# Lançar simulação
roslaunch mobile_rob_dev_sim gazebo.launch

# Lançar navegação simulada
roslaunch mobile_rob_dev_sim navigation_sim.launch
```

## Pacote: envrobotz

Pacote de ambiente e configurações auxiliares.

## API Flask (Legada - NÃO USAR)

A pasta `api/` contém a API Flask **legada** com funcionalidades limitadas:

- Enviar robô para locais (`/ros/goTo/<location>`)
- Listar locais disponíveis
- Verificar status da navegação

**Problemas**:
- Código desatualizado
- Sem Swagger/documentação
- Sem controle de LEDs via Bluetooth

**Solução**: Usar a API moderna **Hugo-Souza/RoboDC_api** (Flask-RESTX v1.2.3) que possui:
- ✅ Swagger UI
- ✅ Controle de LEDs via Bluetooth (ESP32)
- ✅ Endpoints de metadata (`/metadata/version`)
- ✅ Melhor arquitetura (controllers separados)

## Scripts Auxiliares

### `scripts/send_goal.py`

```python
# Enviar objetivo de navegação via actionlib
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.pose.position.x = -37.99  # LE-1
goal.target_pose.pose.position.y = -5.45
goal.target_pose.pose.orientation.w = 1.0

client.send_goal(goal)
client.wait_for_result()
```

### `bash/bag_datehour.sh`

```bash
# Gravar rosbag com timestamp
mkdir -p ~/z_bags/$(date +%Y_%m_%d__%H_%M_%S) && cd $_
rosbag record /cmd_vel /odom /scan /tf /tf_static /map /amcl_pose -o mob_rob
```

## Integração com Outros Repositórios

### Hugo-Souza/RoboDC_api (API REST)

**Instalação**:
```bash
cd ~
git clone https://github.com/Hugo-Souza/RoboDC_api.git
cd RoboDC_api
pip3 install -r requirements.txt
python3 app.py
```

**Endpoints**:
- `GET /ros/available_locals`: Lista 17 locais do DC
- `GET /ros/goTo/{location}`: Envia robô para local
- `GET /ros/status`: Status da navegação
- `GET /ros/cancel`: Cancela navegação
- `GET /led/changeExpression/{expressionNumber}`: Muda expressão LED (0-44)
- `GET /metadata/version`: Versão da API

### thiagoaraujocampos/RoboDC (Aplicativo Móvel)

**Instalação** (no computador de desenvolvimento):
```bash
git clone https://github.com/thiagoaraujocampos/RoboDC.git
cd RoboDC
npm install
ionic serve  # ou ionic capacitor run android
```

**Funcionalidades**:
1. Navegação (17 locais do DC)
2. Cardápio do RU
3. Expressões faciais (45 expressões)
4. Controle manual (joystick virtual)

## Dependências Externas

### Pacotes ROS Necessários

```bash
sudo apt install \
    ros-noetic-navigation \
    ros-noetic-move-base \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-urg-node \
    ros-noetic-tf \
    ros-noetic-robot-state-publisher
```

### Bibliotecas Python (para API)

```bash
pip3 install Flask==2.3.0 Flask-RESTX flask-cors
pip3 install rospkg rospy actionlib
pip3 install pybluez  # Controle Bluetooth ESP32
```

## Workflow de Desenvolvimento

### 1. Modificar Código

```bash
cd ~/laris_wksp/src/ROBO_DC/mobile_rob_dev/src/mobile_rob_dev
nano robotSystem.cpp  # Editar arquivo
```

### 2. Recompilar

```bash
cd ~/laris_wksp
catkin build mobile_rob_dev
source devel/setup.bash
```

### 3. Testar

```bash
rosrun mobile_rob_dev mobile_rob_dev_node
```

### 4. Debug

```bash
# Ver logs
rosnode info /mobile_rob_dev
rostopic echo /odom
rostopic hz /odom

# Ver TF
rosrun tf view_frames
rosrun tf tf_echo odom base_link
```

## Próximos Passos

- [Instalação](./instalacao.md): Configurar ambiente completo
- [Software - Arquitetura ROS1](./software/arquitetura-ros1.md): Detalhes da arquitetura
- [Implantação](./implantacao.md): Deploy em produção

---

**Links Relacionados**:
- 🔗 [Repositório vivaldini/ROBO_DC](https://github.com/vivaldini/ROBO_DC)
- 🔗 [Repositório Hugo-Souza/RoboDC_api](https://github.com/Hugo-Souza/RoboDC_api)
- 🔗 [Repositório thiagoaraujocampos/RoboDC](https://github.com/thiagoaraujocampos/RoboDC)

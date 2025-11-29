---
sidebar_position: 1
---

# Arquitetura ROS 1

## Visão Geral

A arquitetura do robô de 1ª geração é baseada em **ROS 1 Noetic Ninjemys** rodando em Ubuntu 20.04 LTS. O sistema é composto pelo pacote principal `mobile_rob_dev` (do repositório vivaldini/ROBO_DC), que implementa controle de baixo nível, odometria e comunicação serial com o Arduino/Pico, além de pacotes padrão do ROS para navegação (move_base, AMCL) e sensoriamento (Hokuyo LiDAR).

## Arquitetura de Hardware + Software

```
┌─────────────────────────────────────────────────────────────────┐
│                   RASPBERRY PI 4 (ROS MASTER)                   │
│                                                                 │
│  ┌─────────────┐   ┌──────────────┐   ┌────────────────────┐  │
│  │ roscore     │   │ mobile_rob_  │   │ move_base          │  │
│  │ (ROS Master)│◄──┤ dev_node     │◄──┤ (navigation)       │  │
│  └─────────────┘   │ (40 Hz)      │   └────────────────────┘  │
│                    │              │                            │
│                    │ - Serial I/O │   ┌────────────────────┐  │
│                    │ - Odometry   │◄──┤ amcl               │  │
│                    │ - Cmd Vel    │   │ (localization)     │  │
│                    └──────┬───────┘   └────────────────────┘  │
│                           │                                    │
│                           │ /dev/ttyACM0                       │
│                           │ 115200 baud                        │
└───────────────────────────┼────────────────────────────────────┘
                            │ Serial (USB)
┌───────────────────────────▼────────────────────────────────────┐
│               ARDUINO / RASPBERRY PI PICO                       │
│                                                                 │
│  - Leitura de encoders                                          │
│  - Controle PWM dos motores                                     │
│  - Comunicação serial: envia (x, y, theta), recebe (vl, vr)    │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    HOKUYO LIDAR (Ethernet)                      │
│                                                                 │
│  - Conecta diretamente à RPi4 via RJ45                          │
│  - Driver: urg_node ou hokuyo_node                              │
│  - Publica /scan (sensor_msgs/LaserScan)                        │
└─────────────────────────────────────────────────────────────────┘
```

## Pacote Principal: mobile_rob_dev

### Estrutura do Código

```
mobile_rob_dev/
├── src/
│   └── mobile_rob_dev/
│       ├── robotMain.cpp        # Main loop do nó ROS
│       ├── robotSystem.cpp      # Sistema: serial, odometria, ROS
│       └── robot.cpp            # Classe Robot (cinemática, comandos)
├── include/
│   └── mobile_rob_dev/
│       ├── robotSystem.h
│       ├── robot.h
│       └── definitions.h        # Constantes e definições
├── scripts/
│   └── send_goal.py             # Script para enviar goals (actionlib)
├── launch/
│   └── robot.launch             # Launch file principal
├── CMakeLists.txt
└── package.xml
```

### Nó Principal: `mobile_rob_dev_node`

#### Arquivo: robotMain.cpp

```cpp
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile_rob_dev");
    ros::NodeHandle n;
    
    // Instancia sistema do robô
    System system(&n);
    
    // Loop principal a 40 Hz
    ros::Rate loop_rate(40);
    
    while (ros::ok())
    {
        system.step();  // Atualiza sistema
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
```

**Características**:
- Taxa de execução: **40 Hz** (25ms por ciclo)
- Processa callbacks ROS (`ros::spinOnce()`)
- Chama `system.step()` para atualizar estado

#### Arquivo: robotSystem.cpp

Implementa a classe `System`, responsável por:

1. **Comunicação Serial** (`setupSerial()`, `readSerial()`, `writeSerial()`)
   - Porta: `/dev/ttyACM0` (Arduino/Pico via USB)
   - Baud Rate: **115200 bps**
   - Protocolo:
     - **Recebe**: `x,y,theta\n` (posição do robô em metros e radianos)
     - **Envia**: `vl,vr\n` (velocidades das rodas em m/s)

2. **Odometria** (`updateOdometry()`)
   - Calcula odometria diferencial
   - Publica `/odom` (nav_msgs/Odometry)
   - Publica `/pose2d` (geometry_msgs/Pose2D)
   - Broadcast TF: `odom` → `base_link`

3. **Controle de Velocidade** (`cmdVelCallback()`)
   - Subscreve `/robot/cmd_vel` (geometry_msgs/Twist)
   - Converte velocidade linear (vx) e angular (wz) para velocidades das rodas (vl, vr)
   - Usa cinemática diferencial:
     ```
     vl = vx - (wz * wheel_base / 2)
     vr = vx + (wz * wheel_base / 2)
     ```

4. **Parâmetros ROS** (`loadSettings()`)
   - Carrega parâmetros do servidor ROS:
     - `wheel_radius`: Raio da roda (metros)
     - `wheel_base`: Distância entre rodas (metros)
     - `max_linear_velocity`: Velocidade linear máxima (m/s)
     - `max_angular_velocity`: Velocidade angular máxima (rad/s)

**Pseudocódigo de `system.step()`**:
```cpp
void System::step()
{
    // 1. Ler dados do Arduino via serial
    if (readSerial()) {
        // Atualiza pose interna (x, y, theta)
        robot_.updatePose(x, y, theta);
        
        // 2. Calcular e publicar odometria
        updateOdometry();
        
        // 3. Broadcast TF odom -> base_link
        publishTF();
    }
    
    // 4. Enviar comandos de velocidade para Arduino
    writeSerial(vl, vr);
}
```

## Tópicos e Mensagens

### Tópicos de Sensores

| Tópico | Tipo de Mensagem | Frequência | Descrição |
|--------|------------------|------------|-----------|
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | Imagem bruta da câmera |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Informações de calibração |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | Dados do LIDAR |
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | Dados da IMU |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | Odometria do robô |

### Tópicos de Controle

| Tópico | Tipo de Mensagem | Frequência | Descrição |
|--------|------------------|------------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | 10 Hz | Comandos de velocidade |
| `/motor/left/cmd` | `std_msgs/Float64` | 50 Hz | Comando motor esquerdo |
| `/motor/right/cmd` | `std_msgs/Float64` | 50 Hz | Comando motor direito |

### Tópicos de Navegação

| Tópico | Tipo de Mensagem | Frequência | Descrição |
|--------|------------------|------------|-----------|
| `/path` | `nav_msgs/Path` | 1 Hz | Trajetória planejada |
| `/goal` | `geometry_msgs/PoseStamped` | - | Objetivo de navegação |
| `/map` | `nav_msgs/OccupancyGrid` | 0.5 Hz | Mapa de ocupação |

### Tópicos de Estado

| Tópico | Tipo de Mensagem | Frequência | Descrição |
|--------|------------------|------------|-----------|
| `/robot_state` | `robodc_msgs/RobotState` | 5 Hz | Estado atual do robô |
| `/battery_state` | `sensor_msgs/BatteryState` | 1 Hz | Estado da bateria |

## Serviços

| Serviço | Tipo | Descrição |
|---------|------|-----------|
| `/set_mode` | `robodc_srvs/SetMode` | Muda o modo de operação |
| `/calibrate_sensors` | `std_srvs/Trigger` | Inicia calibração de sensores |
| `/emergency_stop` | `std_srvs/Trigger` | Parada de emergência |
| `/reset_odometry` | `std_srvs/Empty` | Reseta odometria |

## Actions

| Action | Tipo | Descrição |
|--------|------|-----------|
| `/navigate_to_goal` | `robodc_msgs/NavigateAction` | Navegar até um objetivo |
| `/follow_path` | `robodc_msgs/FollowPathAction` | Seguir uma trajetória |

## Diagrama de Interação

```
┌──────────────┐     /camera/image_raw     ┌───────────────┐
│ camera_node  ├──────────────────────────►│               │
└──────────────┘                            │               │
                                            │  navigation   │
┌──────────────┐     /scan                 │     node      │
│  lidar_node  ├──────────────────────────►│               │
└──────────────┘                            │               │
                                            └───────┬───────┘
┌──────────────┐     /imu/data                     │
│   imu_node   ├────────────────────┐              │ /cmd_vel
└──────────────┘                    │              │
                                    ▼              ▼
                            ┌────────────────────────────┐
                            │   motor_controller_node    │
                            └──────────┬─────────────────┘
                                       │
                                       │ /odom
                                       ▼
                            ┌─────────────────┐
                            │  Odometry Loop  │
                            └─────────────────┘
```

## Fluxo de Dados

### 1. Percepção
```
Sensores → Nós de Drivers → Processamento → Fusão Sensorial
```

### 2. Decisão
```
Dados Processados → Navegação → Planejamento → Comandos
```

### 3. Ação
```
Comandos → Controle → Drivers de Motor → Atuadores
```

## Parâmetros ROS

### Parâmetros Globais

Definidos no servidor de parâmetros ROS:

- `/robot/name`: Nome do robô
- `/robot/type`: Tipo de robô
- `/robot/max_speed`: Velocidade máxima (m/s)
- `/robot/wheel_base`: Distância entre rodas (m)

### Parâmetros por Nó

#### Base Controller
```yaml
base_controller:
  # Parâmetros físicos do robô
  wheel_separation: 0.30      # metros (distância entre rodas)
  wheel_radius: 0.075         # metros
  encoder_ppr: 1000           # pulsos por rotação
  
  # Limites de velocidade
  max_linear_velocity: 0.5    # m/s
  max_angular_velocity: 1.0   # rad/s
  
  # Controlador PID
  pid_kp: 1.0
  pid_ki: 0.1
  pid_kd: 0.05
  
  # Frequência de publicação
  publish_rate: 50.0          # Hz
```

#### Move Base
```yaml
move_base:
  controller_frequency: 10.0
  planner_frequency: 1.0
  
  # Tolerâncias
  xy_goal_tolerance: 0.1      # metros
  yaw_goal_tolerance: 0.1     # radianos
  
  # Costmap comum
  global_costmap:
    update_frequency: 1.0
    publish_frequency: 0.5
    transform_tolerance: 0.5
    
  local_costmap:
    update_frequency: 5.0
    publish_frequency: 2.0
    transform_tolerance: 0.5
    width: 4.0               # metros
    height: 4.0              # metros
    resolution: 0.05         # metros/célula
    
  # Planejador global (Dijkstra ou A*)
  base_global_planner: "navfn/NavfnROS"
  
  # Planejador local (DWA)
  base_local_planner: "dwa_local_planner/DWAPlannerROS"
  
  DWAPlannerROS:
    max_vel_x: 0.5
    min_vel_x: 0.1
    max_vel_theta: 1.0
    min_vel_theta: 0.4
    
    acc_lim_x: 0.5
    acc_lim_theta: 1.0
    
    sim_time: 2.0
    vx_samples: 10
    vtheta_samples: 20
```

#### Robot Localization (EKF)
```yaml
ekf_localization:
  frequency: 30
  
  odom0: /odom
  odom0_config: [true,  true,  false,    # x, y, z
                 false, false, false,    # roll, pitch, yaw
                 true,  true,  false,    # vx, vy, vz
                 false, false, true,     # vroll, vpitch, vyaw
                 false, false, false]    # ax, ay, az
                 
  imu0: /imu/data
  imu0_config: [false, false, false,
                false, false, true,      # yaw da IMU
                false, false, false,
                false, false, true,      # velocidade angular
                true,  true,  true]      # acelerações
```

#### AMCL
```yaml
amcl:
  odom_model_type: "diff"
  odom_alpha1: 0.2
  odom_alpha2: 0.2
  odom_alpha3: 0.2
  odom_alpha4: 0.2
  
  min_particles: 500
  max_particles: 5000
  
  laser_max_beams: 30
  laser_model_type: "likelihood_field"
  
  update_min_d: 0.1        # metros
  update_min_a: 0.2        # radianos
```

## Transformações (TF)

Árvore de transformações do robô:

```
map
 └── odom
      └── base_link
           ├── base_footprint
           ├── camera_link
           ├── lidar_link
           └── imu_link
```

### Frames Principais

- **map**: Frame do mapa global
- **odom**: Frame de odometria
- **base_link**: Centro do robô
- **camera_link**: Posição da câmera
- **lidar_link**: Posição do LIDAR
- **imu_link**: Posição da IMU

## Configuração de Launch

Exemplo de arquivo launch principal:

```xml
<launch>
  <!-- Parâmetros globais -->
  <param name="robot/name" value="robodc_1gen"/>
  
  <!-- Sensores -->
  <include file="$(find robodc_sensors)/launch/sensors.launch"/>
  
  <!-- Controle -->
  <include file="$(find robodc_control)/launch/control.launch"/>
  
  <!-- Navegação -->
  <include file="$(find robodc_navigation)/launch/navigation.launch"/>
  
  <!-- State Machine -->
  <node name="state_machine" pkg="robodc_control" type="state_machine_node"/>
  
  <!-- TF -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
</launch>
```

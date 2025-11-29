---
sidebar_position: 1
---

# Arquitetura ROS 1

## Visão Geral

A arquitetura do robô de 1ª geração é baseada em ROS 1 (Robot Operating System) e segue o padrão de nós comunicando-se através de tópicos, serviços e actions.

## Nós Principais

### Diagrama de Nós

```
┌─────────────────────────────────────────────────┐
│              ROS Master (roscore)               │
└─────────────────────────────────────────────────┘
           │
           ├──── /camera_node
           │         │ publishes: /camera/image_raw
           │         │            /camera/camera_info
           │
           ├──── /lidar_node
           │         │ publishes: /scan
           │
           ├──── /imu_node
           │         │ publishes: /imu/data
           │
           ├──── /motor_controller
           │         │ subscribes: /cmd_vel
           │         │ publishes: /odom
           │
           ├──── /navigation_node
           │         │ subscribes: /scan, /odom, /camera/image_raw
           │         │ publishes: /cmd_vel, /path
           │
           └──── /state_machine
                     │ publishes: /robot_state
                     │ services: /set_mode
```

### Descrição dos Nós

#### `/rplidar_node`
- **Pacote**: `rplidar_ros`
- **Executável**: `rplidarNode`
- **Função**: Interface com o LIDAR RPLidar A1/A2
- **Taxa de Publicação**: 5.5 Hz (A1) ou 10 Hz (A2)
- **Tópicos Publicados**: `/scan`

#### `/usb_cam` ou `/raspicam_node`
- **Pacote**: `usb_cam` ou `raspicam_node`
- **Executável**: `usb_cam_node` ou `raspicam_node`
- **Função**: Captura e publica imagens da câmera
- **Taxa de Publicação**: 30 Hz (configurável)
- **Tópicos Publicados**: `/camera/image_raw`, `/camera/camera_info`

#### `/imu_node`
- **Pacote**: `imu_filter_madgwick` ou custom
- **Executável**: `imu_filter_node`
- **Função**: Publica dados da IMU com filtro de orientação
- **Taxa de Publicação**: 100 Hz
- **Tópicos Publicados**: `/imu/data`, `/imu/mag` (se disponível)

#### `/base_controller`
- **Pacote**: `robodc_control`
- **Executável**: `base_controller_node`
- **Função**: Controla motores DC via L298N, lê encoders
- **Taxa de Execução**: 50 Hz
- **Tópicos Subscritos**: `/cmd_vel`
- **Tópicos Publicados**: `/odom`

#### `/robot_localization_ekf_node`
- **Pacote**: `robot_localization`
- **Executável**: `ekf_localization_node`
- **Função**: Fusão sensorial (odometria + IMU) via Extended Kalman Filter
- **Taxa de Execução**: 30 Hz
- **Tópicos Subscritos**: `/odom`, `/imu/data`
- **Tópicos Publicados**: `/odometry/filtered`

#### `/move_base`
- **Pacote**: `move_base`
- **Executável**: `move_base`
- **Função**: Navegação autônoma com planejamento global e local
- **Taxa de Execução**: 10 Hz (controller)
- **Actions**: `/move_base` (tipo: `move_base_msgs/MoveBaseAction`)

#### `/amcl`
- **Pacote**: `amcl`
- **Executável**: `amcl`
- **Função**: Localização Monte Carlo Adaptativa
- **Tópicos Subscritos**: `/scan`, `/map`
- **Tópicos Publicados**: `/amcl_pose`, `/particlecloud`

#### `/map_server`
- **Pacote**: `map_server`
- **Executável**: `map_server`
- **Função**: Servidor de mapa estático
- **Tópicos Publicados**: `/map`, `/map_metadata`

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

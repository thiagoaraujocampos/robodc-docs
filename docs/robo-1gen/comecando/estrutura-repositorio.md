---
sidebar_position: 3
---

# Estrutura do Repositório - 1ª Geração

## Visão Geral

O repositório do robô de 1ª geração segue a estrutura padrão de um workspace ROS 1 com Catkin.

## Pastas Principais

```
robodc-1gen/
├── src/
│   ├── robodc_bringup/         # Launch files principais
│   │   ├── launch/
│   │   │   ├── robot.launch
│   │   │   ├── sensors.launch
│   │   │   ├── navigation.launch
│   │   │   └── slam.launch
│   │   └── config/
│   │       ├── costmap_common_params.yaml
│   │       ├── local_costmap_params.yaml
│   │       ├── global_costmap_params.yaml
│   │       └── base_local_planner_params.yaml
│   │
│   ├── robodc_description/     # URDF e modelos
│   │   ├── urdf/
│   │   │   └── robodc.urdf.xacro
│   │   ├── meshes/
│   │   └── launch/
│   │       └── display.launch
│   │
│   ├── robodc_control/         # Controle de baixo nível
│   │   ├── src/
│   │   │   ├── base_controller.cpp
│   │   │   └── motor_driver.cpp
│   │   ├── include/robodc_control/
│   │   └── config/
│   │       └── control_params.yaml
│   │
│   ├── robodc_sensors/         # Drivers de sensores
│   │   ├── launch/
│   │   │   ├── rplidar.launch
│   │   │   ├── camera.launch
│   │   │   └── imu.launch
│   │   └── config/
│   │       └── sensor_params.yaml
│   │
│   ├── robodc_navigation/      # Navegação customizada
│   │   ├── launch/
│   │   ├── config/
│   │   └── maps/
│   │       └── lab_map.yaml
│   │
│   └── robodc_msgs/            # Mensagens customizadas
│       ├── msg/
│       ├── srv/
│       └── action/
│
├── scripts/                     # Scripts auxiliares
│   ├── install_dependencies.sh
│   ├── setup_robot.sh
│   └── calibrate.py
│
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Pacotes ROS

### 1. Pacote robodc_bringup

**Propósito**: Agregador de launch files e configurações principais do sistema

**Arquivos principais**:
- `launch/robot.launch`: Lança todos os sistemas do robô (sensores + controle + localização)
- `launch/sensors.launch`: Apenas sensores (LIDAR, câmera, IMU)
- `launch/navigation.launch`: Stack de navegação (move_base, AMCL)
- `launch/slam.launch`: Sistema SLAM com gmapping ou cartographer

**Configurações**:
- Parâmetros de costmap (global e local)
- Configurações do planejador local (DWA)
- Parâmetros de navegação

### 2. Pacote robodc_description

**Propósito**: Descrição cinemática e visual do robô em URDF/Xacro

**Arquivos principais**:
- `urdf/robodc.urdf.xacro`: Modelo principal com links e joints
- `urdf/sensors.xacro`: Sensores (LIDAR, câmera, IMU)
- `urdf/gazebo.xacro`: Plugins para simulação Gazebo
- `meshes/`: Modelos 3D (.stl, .dae)

**Frames definidos**:
- `base_link`: Centro da base do robô
- `base_footprint`: Projeção no chão
- `laser_frame`: Frame do LIDAR
- `camera_link`: Frame da câmera
- `imu_link`: Frame da IMU

### 3. Pacote robodc_control

**Propósito**: Controle de baixo nível dos motores e odometria

**Nós principais**:
- `base_controller`: 
  - Subscreve `/cmd_vel`
  - Controla motores DC via GPIO
  - Lê encoders
  - Publica `/odom` e TF `odom -> base_link`

**Implementação**:
- C++ para performance em tempo real
- Interface com GPIO via pigpio ou WiringPi
- Controle PWM para motores
- Leitura de encoders por interrupção

### 4. Pacote robodc_sensors

**Propósito**: Launch files e configurações para sensores

**Conteúdo**:
- `launch/rplidar.launch`: 
  - Inicia `rplidar_node` do pacote `rplidar_ros`
  - Configura porta serial (/dev/ttyUSB0)
  - Define frame_id como `laser_frame`
  
- `launch/camera.launch`:
  - Inicia `usb_cam` ou `raspicam_node`
  - Configura resolução e framerate
  - Publica imagem raw e camera_info

- `launch/imu.launch`:
  - Inicia driver da IMU (I2C)
  - Aplica filtro Madgwick para orientação
  - Publica `/imu/data`

### 5. Pacote robodc_navigation

**Propósito**: Configurações específicas de navegação e mapas

**Conteúdo**:
- `maps/`: Mapas salvos (.yaml + .pgm)
- `launch/amcl.launch`: Localização com mapa pré-existente
- `launch/gmapping.launch`: SLAM para criar novos mapas
- Custom recovery behaviors (opcional)

### 6. Pacote robodc_msgs

**Propósito**: Definições de mensagens, serviços e actions customizadas

**Exemplos**:
```
msg/
├── RobotState.msg          # Estado geral do robô
├── BatteryStatus.msg       # Estado da bateria
└── EncoderCounts.msg       # Contagem bruta dos encoders

srv/
├── SetMode.srv             # Mudar modo de operação
└── ResetOdometry.srv       # Resetar odometria

action/
└── GoToGoal.action         # Navegar para objetivo
```

## Mensagens e Serviços

### Mensagens Customizadas (`msg/`)
- `RobotState.msg`: Estado do robô
- `SensorData.msg`: Dados dos sensores
- [Outras mensagens]

### Serviços Customizados (`srv/`)
- `SetMode.srv`: Mudar modo de operação
- `Calibrate.srv`: Calibrar sensores
- [Outros serviços]

### Actions (`action/`)
- `Navigate.action`: Ação de navegação
- [Outras actions]

## Arquivos de Launch

### Launch Files Principais

#### `robot.launch`
Lança todos os sistemas do robô real
```xml
<launch>
  <!-- Argumentos -->
  <arg name="use_sim_time" default="false"/>
  
  <!-- Robot Description -->
  <param name="robot_description" 
         command="$(find xacro)/xacro '$(find robodc_description)/urdf/robodc.urdf.xacro'"/>
  
  <!-- Robot State Publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
  
  <!-- Sensores -->
  <include file="$(find robodc_bringup)/launch/sensors.launch"/>
  
  <!-- Base Controller -->
  <node name="base_controller" pkg="robodc_control" type="base_controller_node" output="screen">
    <rosparam file="$(find robodc_control)/config/control_params.yaml" command="load"/>
  </node>
  
  <!-- Robot Localization (EKF) -->
  <node name="ekf_localization" pkg="robot_localization" type="ekf_localization_node">
    <rosparam file="$(find robodc_bringup)/config/ekf_params.yaml" command="load"/>
  </node>
</launch>
```

#### `sensors.launch`
Lança apenas os sensores
```xml
<launch>
  <!-- RPLidar -->
  <node name="rplidar_node" pkg="rplidar_ros" type="rplidarNode" output="screen">
    <param name="serial_port" value="/dev/ttyUSB0"/>
    <param name="serial_baudrate" value="115200"/>
    <param name="frame_id" value="laser_frame"/>
    <param name="inverted" value="false"/>
    <param name="angle_compensate" value="true"/>
  </node>
  
  <!-- Câmera USB -->
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
    <param name="video_device" value="/dev/video0"/>
    <param name="image_width" value="640"/>
    <param name="image_height" value="480"/>
    <param name="pixel_format" value="yuyv"/>
    <param name="camera_frame_id" value="camera_link"/>
    <param name="framerate" value="30"/>
  </node>
  
  <!-- IMU -->
  <node name="imu_node" pkg="imu_filter_madgwick" type="imu_filter_node">
    <param name="use_mag" value="false"/>
    <param name="publish_tf" value="false"/>
    <param name="world_frame" value="enu"/>
    <remap from="/imu/data_raw" to="/imu/data"/>
  </node>
</launch>
```

#### `navigation.launch`
Lança apenas o stack de navegação
```xml
<launch>
  <arg name="map_file" default="$(find robodc_navigation)/maps/lab_map.yaml"/>
  
  <!-- Map Server -->
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)"/>
  
  <!-- AMCL -->
  <node name="amcl" pkg="amcl" type="amcl">
    <rosparam file="$(find robodc_bringup)/config/amcl_params.yaml" command="load"/>
  </node>
  
  <!-- Move Base -->
  <node name="move_base" pkg="move_base" type="move_base" respawn="false" output="screen">
    <rosparam file="$(find robodc_bringup)/config/costmap_common_params.yaml" command="load" ns="global_costmap"/>
    <rosparam file="$(find robodc_bringup)/config/costmap_common_params.yaml" command="load" ns="local_costmap"/>
    <rosparam file="$(find robodc_bringup)/config/local_costmap_params.yaml" command="load"/>
    <rosparam file="$(find robodc_bringup)/config/global_costmap_params.yaml" command="load"/>
    <rosparam file="$(find robodc_bringup)/config/base_local_planner_params.yaml" command="load"/>
  </node>
</launch>
```

#### `slam.launch`
Lança SLAM para criar mapas
```xml
<launch>
  <!-- GMapping -->
  <node name="gmapping" pkg="gmapping" type="slam_gmapping" output="screen">
    <param name="map_update_interval" value="2.0"/>
    <param name="maxUrange" value="10.0"/>
    <param name="sigma" value="0.05"/>
    <param name="kernelSize" value="1"/>
    <param name="lstep" value="0.05"/>
    <param name="astep" value="0.05"/>
    <param name="iterations" value="5"/>
    <param name="lsigma" value="0.075"/>
    <param name="ogain" value="3.0"/>
    <param name="lskip" value="0"/>
    <param name="minimumScore" value="50"/>
    <param name="srr" value="0.1"/>
    <param name="srt" value="0.2"/>
    <param name="str" value="0.1"/>
    <param name="stt" value="0.2"/>
    <param name="linearUpdate" value="0.5"/>
    <param name="angularUpdate" value="0.5"/>
    <param name="temporalUpdate" value="3.0"/>
    <param name="resampleThreshold" value="0.5"/>
    <param name="particles" value="30"/>
    <param name="xmin" value="-50.0"/>
    <param name="ymin" value="-50.0"/>
    <param name="xmax" value="50.0"/>
    <param name="ymax" value="50.0"/>
    <param name="delta" value="0.05"/>
    <param name="llsamplerange" value="0.01"/>
    <param name="llsamplestep" value="0.01"/>
    <param name="lasamplerange" value="0.005"/>
    <param name="lasamplestep" value="0.005"/>
  </node>
</launch>
```

## Configurações

### Arquivos YAML Importantes

- `config/robot_params.yaml`: Parâmetros do robô
- `config/sensor_params.yaml`: Configuração dos sensores
- `config/navigation_params.yaml`: Parâmetros de navegação
- `config/control_params.yaml`: Parâmetros de controle

## Scripts Auxiliares

- `scripts/install_dependencies.sh`: Instala dependências
- `scripts/calibrate_sensors.py`: Calibração de sensores
- `scripts/run_tests.sh`: Executa testes
- `scripts/deploy.sh`: Script de implantação

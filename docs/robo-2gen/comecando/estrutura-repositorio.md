---
sidebar_position: 3
---

# Estrutura do Repositório - 2ª Geração

## Visão Geral

O repositório do robô de 2ª geração segue a estrutura de um workspace ROS 2 com colcon, diferente da estrutura catkin da 1ª geração.

## Estrutura Principal

```
ros2_ws/
├── src/
│   └── robodc-2gen/
│       ├── robodc_bringup/          # Launch files principais
│       ├── robodc_description/      # URDF, meshes, modelos
│       ├── robodc_navigation/       # Nav2 e navegação
│       ├── robodc_perception/       # Sensores e percepção
│       ├── robodc_control/          # Controle e drivers
│       ├── robodc_interfaces/       # Msgs, Srvs, Actions
│       ├── robodc_behaviors/        # Behavior trees
│       ├── robodc_simulation/       # Gazebo, mundos
│       └── robodc_common/           # Utilitários compartilhados
├── build/                           # Arquivos de build (não versionado)
├── install/                         # Pacotes instalados
└── log/                             # Logs de compilação
```

## Pacotes ROS 2

### 1. `robodc_bringup/`
**Propósito**: Launch files e configurações principais

```
robodc_bringup/
├── launch/
│   ├── robot.launch.py              # Lança robô real
│   ├── simulation.launch.py         # Lança simulação
│   ├── navigation.launch.py         # Apenas navegação
│   └── sensors.launch.py            # Apenas sensores
├── config/
│   ├── robot_params.yaml
│   ├── sensor_params.yaml
│   └── nav2_params.yaml
├── package.xml
└── CMakeLists.txt
```

### 2. `robodc_description/`
**Propósito**: Descrição do robô (URDF, meshes)

```
robodc_description/
├── urdf/
│   ├── robodc.urdf.xacro            # Descrição principal
│   ├── sensors.urdf.xacro
│   └── gazebo.urdf.xacro
├── meshes/
│   ├── base/
│   ├── sensors/
│   └── wheels/
├── rviz/
│   └── default.rviz
├── launch/
│   └── display.launch.py
├── package.xml
└── CMakeLists.txt
```

### 3. `robodc_navigation/`
**Propósito**: Navegação autônoma com Nav2

```
robodc_navigation/
├── robodc_navigation/              # Código Python
│   ├── __init__.py
│   ├── path_planner.py
│   ├── obstacle_avoider.py
│   └── waypoint_follower.py
├── launch/
│   ├── navigation.launch.py
│   ├── slam.launch.py
│   └── localization.launch.py
├── config/
│   ├── nav2_params.yaml
│   ├── slam_params.yaml
│   └── planner_params.yaml
├── maps/                            # Mapas salvos
│   ├── map1.yaml
│   └── map1.pgm
├── behavior_trees/                  # BT XMLs
│   └── navigate_to_pose.xml
├── test/
│   └── test_navigation.py
├── package.xml
├── CMakeLists.txt
└── setup.py
```

### 4. `robodc_perception/`
**Propósito**: Processamento de sensores

```
robodc_perception/
├── robodc_perception/
│   ├── camera_processor.py
│   ├── lidar_processor.py
│   ├── sensor_fusion.py
│   └── object_detector.py
├── launch/
│   ├── sensors.launch.py
│   ├── camera.launch.py
│   └── lidar.launch.py
├── config/
│   ├── camera_calibration.yaml
│   └── sensor_config.yaml
├── test/
├── package.xml
├── CMakeLists.txt
└── setup.py
```

### 5. `robodc_control/`
**Propósito**: Controle de baixo nível

```
robodc_control/
├── include/robodc_control/
│   ├── motor_controller.hpp
│   └── pid_controller.hpp
├── src/
│   ├── motor_controller.cpp
│   ├── motor_controller_node.cpp
│   └── pid_controller.cpp
├── launch/
│   └── control.launch.py
├── config/
│   ├── control_params.yaml
│   └── ros2_control.yaml
├── test/
├── package.xml
└── CMakeLists.txt
```

### 6. `robodc_interfaces/`
**Propósito**: Definições customizadas

```
robodc_interfaces/
├── msg/
│   ├── RobotState.msg
│   ├── SensorData.msg
│   └── NavigationStatus.msg
├── srv/
│   ├── SetMode.srv
│   ├── Calibrate.srv
│   └── SaveMap.srv
├── action/
│   ├── NavigateToGoal.action
│   └── FollowPath.action
├── package.xml
└── CMakeLists.txt
```

### 7. `robodc_behaviors/`
**Propósito**: Behavior Trees para decisões complexas

```
robodc_behaviors/
├── behavior_trees/
│   ├── explore.xml
│   ├── patrol.xml
│   └── emergency.xml
├── robodc_behaviors/
│   ├── custom_nodes.py
│   └── behavior_executor.py
├── launch/
│   └── behaviors.launch.py
├── config/
├── package.xml
└── setup.py
```

### 8. `robodc_simulation/`
**Propósito**: Simulação Gazebo

```
robodc_simulation/
├── worlds/
│   ├── empty.world
│   ├── office.world
│   └── warehouse.world
├── models/                          # Modelos Gazebo
├── launch/
│   ├── gazebo.launch.py
│   └── spawn_robot.launch.py
├── config/
├── package.xml
└── CMakeLists.txt
```

## Diferenças Principais vs 1ª Geração

| Aspecto | 1ª Geração | 2ª Geração |
|---------|------------|------------|
| **Build Tool** | catkin_make | colcon |
| **Launch Files** | XML (.launch) | Python (.launch.py) |
| **Nós** | Standalone | Pode usar Lifecycle/Component |
| **Parâmetros** | Servidor central | Arquivo + override CLI |
| **Messages** | Em cada pacote | Pacote centralizado *_interfaces |
| **Estrutura** | src/ diretamente | Workspace com src/ |

## Arquivos de Configuração Importantes

### package.xml (ROS 2 Format)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robodc_navigation</name>
  <version>2.0.0</version>
  <description>Navigation package for RoboDC 2nd generation</description>
  
  <maintainer email="dev@robodc.com">RoboDC Team</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>nav2_bringup</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### setup.py (para pacotes Python)

```python
from setuptools import setup
from glob import glob
import os

package_name = 'robodc_navigation'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboDC Team',
    maintainer_email='dev@robodc.com',
    description='Navigation for RoboDC 2gen',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = robodc_navigation.navigation_node:main',
            'waypoint_follower = robodc_navigation.waypoint_follower:main',
        ],
    },
)
```

## Launch Files (Python)

### Exemplo: robot.launch.py

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Incluir sensors
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('robodc_bringup'),
                'launch',
                'sensors.launch.py'
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        # Controle
        Node(
            package='robodc_control',
            executable='motor_controller',
            name='motor_controller',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
```

## Scripts Auxiliares

```
scripts/
├── install_dependencies.sh          # Instala deps
├── setup_robot.sh                   # Config inicial
├── calibrate_sensors.py             # Calibração
├── run_tests.sh                     # Testes
├── create_map.sh                    # SLAM e salvar mapa
└── deploy.sh                        # Deploy no robô
```

## Convenções de Código

### Python (seguir PEP 8)
- Usar snake_case para funções e variáveis
- Classes em PascalCase
- Docstrings para todas funções públicas

### C++ (seguir ROS 2 Style Guide)
- snake_case para funções
- PascalCase para classes
- Comentários Doxygen

## Testes

```
test/
├── unit/                            # Testes unitários
├── integration/                     # Testes de integração
└── system/                          # Testes de sistema
```

Executar testes:
```bash
colcon test --packages-select robodc_navigation
colcon test-result --all --verbose
```

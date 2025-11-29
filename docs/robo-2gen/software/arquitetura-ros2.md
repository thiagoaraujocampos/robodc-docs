---
sidebar_position: 1
---

# Arquitetura ROS 2

## Visão Geral

A arquitetura da 2ª geração utiliza ROS 2 Humble com comunicação DDS e suporte a lifecycle nodes.

## Principais Diferenças vs ROS 1

| Aspecto | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Comunicação** | TCPROS | DDS (Data Distribution Service) |
| **Master** | Centralizado (roscore) | Descentralizado |
| **Tempo Real** | Limitado | Suporte nativo |
| **QoS** | Fixo | Configurável |
| **Segurança** | Limitada | Nativa (SROS2) |
| **Linguagens** | Principalmente C++/Python | Múltiplas (C++, Python, Rust, etc) |

## Nós Principais

### Diagrama de Arquitetura

```
┌────────────────────────────────────────────┐
│         DDS Domain (sem Master)            │
└────────────────────────────────────────────┘
           │
           ├──── /camera_node (lifecycle)
           │         │ publishes: /camera/image_raw
           │         │            /camera/camera_info
           │
           ├──── /lidar_node (lifecycle)
           │         │ publishes: /scan
           │
           ├──── /imu_node
           │         │ publishes: /imu/data
           │
           ├──── /controller_manager
           │         │ manages: controllers
           │         │ publishes: /odom
           │
           ├──── /bt_navigator (Nav2)
           │         │ uses: behavior trees
           │         │ actions: /navigate_to_pose
           │
           └──── /lifecycle_manager
                     │ manages: node states
```

### Nós Lifecycle

ROS 2 permite gerenciar estados dos nós:

```
Unconfigured → Configure → Inactive → Activate → Active
                                ↓                   ↓
                           Cleanup          Deactivate
```

Exemplo de nó lifecycle:
```python
from rclpy.lifecycle import Node, State, TransitionCallbackReturn

class MyLifecycleNode(Node):
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Configurar recursos
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Ativar publicação/subscrição
        return TransitionCallbackReturn.SUCCESS
```

## Tópicos e QoS

### Configuração de QoS

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# QoS para sensores (best effort, volatile)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# QoS para comandos (reliable, transient local)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

### Tópicos Principais

| Tópico | Tipo | QoS | Frequência | Descrição |
|--------|------|-----|------------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | Sensor | 10 Hz | LIDAR |
| `/camera/image_raw` | `sensor_msgs/Image` | Sensor | 30 Hz | Câmera |
| `/imu/data` | `sensor_msgs/Imu` | Sensor | 100 Hz | IMU |
| `/odom` | `nav_msgs/Odometry` | Reliable | 50 Hz | Odometria |
| `/cmd_vel` | `geometry_msgs/Twist` | Reliable | 10 Hz | Comandos |
| `/map` | `nav_msgs/OccupancyGrid` | Transient | 1 Hz | Mapa |

## Serviços e Actions

### Serviços ROS 2

```python
# Exemplo de serviço
from example_interfaces.srv import SetBool

def set_mode_callback(request, response):
    response.success = True
    response.message = "Mode changed"
    return response

service = node.create_service(SetBool, 'set_mode', set_mode_callback)
```

### Actions (Nav2)

```python
from nav2_msgs.action import NavigateToPose

# Cliente de action
action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

# Enviar goal
goal_msg = NavigateToPose.Goal()
goal_msg.pose.header.frame_id = 'map'
goal_msg.pose.pose.position.x = 2.0
goal_msg.pose.pose.position.y = 1.0

action_client.send_goal_async(goal_msg)
```

## Nav2 (Navigation 2)

### Arquitetura Nav2

```
┌─────────────────────────────────────────┐
│           BT Navigator                  │
│  (Behavior Tree based navigation)      │
└──────────────┬──────────────────────────┘
               │
    ┌──────────┴──────────┐
    │                     │
┌───▼────┐        ┌──────▼─────┐
│Planner │        │ Controller │
│Server  │        │   Server   │
└───┬────┘        └──────┬─────┘
    │                    │
    └────────┬───────────┘
             │
    ┌────────▼────────┐
    │  Costmap 2D     │
    │  (local/global) │
    └─────────────────┘
```

### Behavior Trees

Nav2 usa behavior trees para lógica complexa:

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <ComputePathToPose goal="{goal}"/>
      <FollowPath path="{path}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

## Components (Composable Nodes)

ROS 2 permite compor múltiplos nós em um processo:

```python
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

# Criar múltiplos componentes em um container
executor = SingleThreadedExecutor()
executor.add_node(node1)
executor.add_node(node2)
executor.spin()
```

Benefícios:
- Menor overhead de comunicação
- Melhor performance
- Uso mais eficiente de recursos

## Parâmetros

### Arquivo de Parâmetros YAML

```yaml
/**:
  ros__parameters:
    use_sim_time: false
    
navigation_node:
  ros__parameters:
    max_speed: 1.0
    goal_tolerance: 0.1
    controller_frequency: 10.0
```

### Carregar Parâmetros

```bash
ros2 run pkg node --ros-args --params-file params.yaml
```

## Transformações (TF2)

Árvore TF similar à 1ª geração, mas com melhor performance:

```
map
 └── odom
      └── base_link
           ├── base_footprint
           ├── camera_link
           │    └── camera_optical_frame
           ├── lidar_link
           └── imu_link
```

### Publicar TF2

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

broadcaster = TransformBroadcaster(node)

t = TransformStamped()
t.header.stamp = node.get_clock().now().to_msg()
t.header.frame_id = 'odom'
t.child_frame_id = 'base_link'
# ... preencher transform

broadcaster.sendTransform(t)
```

## Launch System (Python)

### Launch File Exemplo

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        Node(
            package='robodc_perception',
            executable='camera_node',
            name='camera',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/image', '/camera/image_raw')],
            condition=IfCondition(LaunchConfiguration('enable_camera'))
        ),
    ])
```

## Diagnósticos

### Publicar Diagnósticos

```python
from diagnostic_updater import Updater, DiagnosticStatusWrapper

updater = Updater(node)
updater.setHardwareID("robodc_2gen")

def check_sensor(stat: DiagnosticStatusWrapper):
    if sensor_ok:
        stat.summary(DiagnosticStatusWrapper.OK, "Sensor working")
    else:
        stat.summary(DiagnosticStatusWrapper.ERROR, "Sensor failure")
    stat.add("Temperature", temp)
    return stat

updater.add("Sensor Check", check_sensor)
```

## Comparação de Performance

### Latência

| Cenário | ROS 1 | ROS 2 | Melhoria |
|---------|-------|-------|----------|
| Intra-process | ~10 ms | ~1 ms | 10x |
| Inter-process | ~5 ms | ~3 ms | 1.7x |

### Throughput

| Tipo de Dados | ROS 1 | ROS 2 | Melhoria |
|---------------|-------|-------|----------|
| Pequenas msgs | X MB/s | Y MB/s | Z% |
| Grandes msgs | X MB/s | Y MB/s | Z% |

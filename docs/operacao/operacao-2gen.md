---
sidebar_position: 2
---

# Como Operar o Robô de 2ª Geração

## Diferenças em Relação à 1ª Geração

A operação da 2ª geração é similar à 1ª, mas com comandos ROS 2:

| Operação | 1ª Geração (ROS 1) | 2ª Geração (ROS 2) |
|----------|-------------------|-------------------|
| Listar tópicos | `rostopic list` | `ros2 topic list` |
| Listar nós | `rosnode list` | `ros2 node list` |
| Echo tópico | `rostopic echo /topic` | `ros2 topic echo /topic` |
| Chamar serviço | `rosservice call /srv` | `ros2 service call /srv` |
| Launch | `roslaunch pkg file.launch` | `ros2 launch pkg file.launch.py` |

## Preparação Pré-Operação

[Similar à 1ª geração - ver seção anterior]

## Iniciar o Software

### Método 1: Inicialização Automática

```bash
# Verificar serviço
systemctl status robodc2
```

### Método 2: Inicialização Manual

```bash
# Source do ambiente ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Lançar o robô
ros2 launch robodc_bringup robot.launch.py
```

## Modos de Operação

### Modo Manual (Teleoperação)

```bash
# Teclado
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel

# Joystick (se configurado)
ros2 launch robodc_bringup teleop_joystick.launch.py
```

### Modo Autônomo (Nav2)

```bash
# Lançar Nav2
ros2 launch robodc_navigation navigation.launch.py

# Enviar objetivo via comando
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}"

# Ou usar RViz2 "Nav2 Goal" button
```

### Modo Behavior Tree

```bash
# Executar behavior tree específico
ros2 launch robodc_behaviors execute_bt.launch.py bt_xml:=patrol.xml
```

## Monitoramento

### Verificar Status

```bash
# Nós ativos
ros2 node list

# Tópicos
ros2 topic list

# Informações de um tópico
ros2 topic info /scan -v

# Frequência de publicação
ros2 topic hz /scan

# Bandwidth
ros2 topic bw /scan
```

### Usar RViz2

```bash
ros2 launch robodc_bringup rviz.launch.py

# Ou abrir configuração específica
rviz2 -d ~/ros2_ws/src/robodc-2gen/robodc_description/rviz/navigation.rviz
```

### Diagnósticos

```bash
# Ver diagnósticos
ros2 topic echo /diagnostics

# Usar ferramentas rqt
ros2 run rqt_console rqt_console  # Console de logs
ros2 run rqt_graph rqt_graph      # Grafo de nós
ros2 run rqt_robot_monitor rqt_robot_monitor  # Monitor
```

## Navegação com Nav2

### Inicializar Localização (AMCL)

```bash
# Carregar mapa
ros2 launch robodc_navigation localization.launch.py map:=/path/to/map.yaml

# Definir pose inicial em RViz2:
# Clicar em "2D Pose Estimate"
# Clicar e arrastar no mapa na posição aproximada do robô
```

### Criar Mapa (SLAM Toolbox)

```bash
# Iniciar SLAM
ros2 launch robodc_navigation slam.launch.py

# Teleoperar para explorar ambiente
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Salvar mapa
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: meu_mapa}}"

# Mapa será salvo em ~/maps/
```

### Seguir Waypoints

```bash
# Carregar e seguir waypoints
ros2 launch robodc_navigation follow_waypoints.launch.py \
  waypoints_file:=waypoints.yaml
```

### Patrulha

```bash
# Modo patrulha entre pontos
ros2 launch robodc_navigation patrol.launch.py \
  patrol_points:=patrol.yaml loop:=true
```

## Lifecycle Management (ROS 2)

ROS 2 permite gerenciar ciclo de vida dos nós:

```bash
# Ver estado de um nó lifecycle
ros2 lifecycle get /navigation_node

# Configurar nó
ros2 lifecycle set /navigation_node configure

# Ativar nó
ros2 lifecycle set /navigation_node activate

# Desativar nó
ros2 lifecycle set /navigation_node deactivate

# Limpar nó
ros2 lifecycle set /navigation_node cleanup
```

## Gravação e Reprodução (rosbag2)

### Gravar Dados

```bash
# Gravar todos os tópicos
ros2 bag record -a

# Gravar tópicos específicos
ros2 bag record /scan /camera/image_raw /odom

# Gravar com filtro
ros2 bag record -a -x "/camera/.*"  # Exclui tópicos de câmera
```

### Reproduzir Dados

```bash
# Listar bags
ros2 bag list

# Ver informações
ros2 bag info rosbag2_2024_11_28-10_00_00

# Reproduzir
ros2 bag play rosbag2_2024_11_28-10_00_00

# Reproduzir em loop
ros2 bag play rosbag2_2024_11_28-10_00_00 --loop
```

## Parâmetros Dinâmicos

### Listar Parâmetros

```bash
# Ver parâmetros de um nó
ros2 param list /navigation_node

# Ver valor de um parâmetro
ros2 param get /navigation_node max_speed
```

### Modificar Parâmetros

```bash
# Modificar em tempo de execução
ros2 param set /navigation_node max_speed 0.5

# Salvar parâmetros atuais
ros2 param dump /navigation_node > navigation_params.yaml
```

## Parada e Desligamento

### Parada de Emergência

```bash
# Via serviço
ros2 service call /emergency_stop std_srvs/srv/Trigger

# Ou parar publicação de cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

### Desligamento Graceful

```bash
# Parar launch (Ctrl+C)
# Ou desligar nós lifecycle
ros2 lifecycle set /node_name deactivate
ros2 lifecycle set /node_name cleanup
ros2 lifecycle set /node_name shutdown
```

### Desligar Sistema

```bash
# Desligar ROS 2 daemon
ros2 daemon stop

# Desligar computador
sudo shutdown -h now
```

## QoS (Quality of Service)

Ver políticas QoS de um tópico:

```bash
ros2 topic info /scan -v
```

Exemplo de QoS:
- **Reliability**: RELIABLE vs BEST_EFFORT
- **Durability**: VOLATILE vs TRANSIENT_LOCAL
- **History**: KEEP_LAST vs KEEP_ALL

## Multi-Robot (se aplicável)

```bash
# Usar namespaces
ros2 launch robodc_bringup robot.launch.py namespace:=/robot1

# Ou usar ROS_DOMAIN_ID diferente
export ROS_DOMAIN_ID=1
ros2 launch robodc_bringup robot.launch.py
```

## Troubleshooting Operacional

### Problemas de Comunicação DDS

```bash
# Verificar middleware
echo $RMW_IMPLEMENTATION

# Diagnóstico completo
ros2 doctor --report

# Reiniciar daemon
ros2 daemon stop
ros2 daemon start
```

### Nav2 Não Planeja Rota

**Verificações**:
```bash
# Ver logs do planner
ros2 topic echo /local_costmap/costmap_updates

# Verificar parâmetros
ros2 param list /bt_navigator

# Recarregar mapa
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
  "{map_url: /path/to/map.yaml}"
```

### TF não Disponível

```bash
# Ver árvore TF
ros2 run tf2_tools view_frames

# Echo transformação específica
ros2 run tf2_ros tf2_echo map base_link
```

## Boas Práticas ROS 2

- ✅ Usar QoS apropriado para cada tópico
- ✅ Implementar lifecycle nodes quando apropriado
- ✅ Usar composable nodes para melhor performance
- ✅ Configurar DDS corretamente (Cyclone recomendado)
- ✅ Usar ROS_DOMAIN_ID único para cada robô
- ✅ Monitorar latência e bandwidth

## Performance Tips

- Usar Cyclone DDS em vez de Fast DDS
- Configurar QoS adequadamente
- Usar `ros2 bag` com compressão
- Limitar taxa de publicação de sensores
- Usar intra-process communication quando possível

## Comandos Úteis Rápidos

```bash
# Status geral
ros2 doctor

# Introspection
ros2 node info /node_name
ros2 topic info /topic_name
ros2 service list
ros2 action list

# Executar nó com remapping
ros2 run pkg node --ros-args --remap old:=new

# Setar parâmetro ao lançar
ros2 run pkg node --ros-args -p param:=value
```

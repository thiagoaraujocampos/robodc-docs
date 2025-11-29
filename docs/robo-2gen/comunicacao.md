---
sidebar_position: 3
---

# Comunicação - 2ª Geração

## Comunicação Interna entre Nós

### DDS (Data Distribution Service)

ROS 2 utiliza DDS como middleware de comunicação.

#### Implementações DDS Disponíveis

- **Fast DDS** (padrão)
- **Cyclone DDS** (recomendado para melhor performance)
- **RTI Connext DDS**

#### Configurar RMW

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Intra-Process Communication

Comunicação otimizada dentro do mesmo processo:

```python
# Habilitar intra-process
node = Node('my_node', enable_intra_process_comms=True)
```

Benefícios:
- Zero-copy quando possível
- Redução significativa de latência
- Menor uso de CPU

## QoS (Quality of Service)

### Políticas de QoS

#### Reliability
- **RELIABLE**: Garante entrega (TCP-like)
- **BEST_EFFORT**: Não garante (UDP-like)

#### Durability
- **VOLATILE**: Apenas para novos subscribers
- **TRANSIENT_LOCAL**: Últimas N mensagens salvas

#### History
- **KEEP_LAST**: Mantém últimas N mensagens
- **KEEP_ALL**: Mantém todas (até limite de recursos)

### Exemplo de Configuração

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# Para sensores (latência crítica)
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10
)

# Para comandos (entrega garantida)
command_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1
)

# Criar subscription com QoS
subscription = node.create_subscription(
    LaserScan,
    '/scan',
    callback,
    sensor_qos
)
```

## Comunicação Multi-Robô

### Usando ROS_DOMAIN_ID

```bash
# Robô 1
export ROS_DOMAIN_ID=1
ros2 launch robodc_bringup robot.launch.py

# Robô 2
export ROS_DOMAIN_ID=2
ros2 launch robodc_bringup robot.launch.py
```

### Usando Namespaces

```bash
ros2 launch robodc_bringup robot.launch.py namespace:=/robot1
ros2 launch robodc_bringup robot.launch.py namespace:=/robot2
```

## Segurança (SROS2)

### Habilitar Segurança

```bash
# Criar keystore
ros2 security create_keystore demo_keys

# Criar chaves para nó
ros2 security create_key demo_keys /my_node

# Executar com segurança
export ROS_SECURITY_KEYSTORE=~/demo_keys
export ROS_SECURITY_ENABLE=true
ros2 run pkg node
```

## Monitoramento de Comunicação

### Ferramentas de Análise

```bash
# Ver bandwidth de tópico
ros2 topic bw /scan

# Ver frequência
ros2 topic hz /scan

# Ver latência (com echo)
ros2 topic echo /scan --field header.stamp

# Informações detalhadas
ros2 topic info /scan -v
```

### Diagnóstico de Rede

```bash
# Doctor para diagnóstico geral
ros2 doctor

# Verificar peers
ros2 daemon status

# Ver nós descobertos
ros2 node list
```

## Otimização de Performance

### Best Practices

1. **Usar QoS apropriado**
   - BEST_EFFORT para sensores
   - RELIABLE para comandos críticos

2. **Intra-process quando possível**
   - Composable nodes
   - Component containers

3. **Limitar taxa de publicação**
   ```python
   # Publicar a 10 Hz em vez de loop infinito
   timer = node.create_timer(0.1, timer_callback)
   ```

4. **Usar Cyclone DDS**
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```

5. **Configurar DDS para rede**
   ```xml
   <!-- cyclonedds.xml -->
   <CycloneDDS>
     <Domain>
       <General>
         <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
       </General>
     </Domain>
   </CycloneDDS>
   ```

## Latência e Confiabilidade

### Medição de Latência

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

class LatencyNode(Node):
    def __init__(self):
        super().__init__('latency_node')
        self.subscription = self.create_subscription(
            Header,
            '/test_topic',
            self.callback,
            10
        )
    
    def callback(self, msg):
        now = self.get_clock().now()
        sent_time = rclpy.time.Time.from_msg(msg.stamp)
        latency = (now - sent_time).nanoseconds / 1e6  # ms
        self.get_logger().info(f'Latency: {latency:.2f} ms')
```

### Garantir Confiabilidade

- Usar RELIABLE QoS para mensagens críticas
- Implementar heartbeats/watchdogs
- Logging adequado
- Tratamento de exceções

## Troubleshooting

### Nós não se Descobrem

**Causas comuns**:
- ROS_DOMAIN_ID diferente
- Firewall bloqueando multicast
- Interface de rede errada

**Soluções**:
```bash
# Verificar domain
echo $ROS_DOMAIN_ID

# Verificar daemon
ros2 daemon status
ros2 daemon stop
ros2 daemon start

# Testar comunicação
ros2 run demo_nodes_cpp talker  # Terminal 1
ros2 run demo_nodes_cpp listener  # Terminal 2
```

### Alta Latência

**Soluções**:
- Mudar para Cyclone DDS
- Usar intra-process communication
- Ajustar QoS (BEST_EFFORT para sensores)
- Verificar sobrecarga de CPU

### Perda de Mensagens

**Soluções**:
- Aumentar depth do QoS
- Usar RELIABLE em vez de BEST_EFFORT
- Verificar bandwidth da rede
- Reduzir taxa de publicação

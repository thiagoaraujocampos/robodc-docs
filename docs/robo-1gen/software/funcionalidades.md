---
sidebar_position: 2
---

# Funcionalidades Principais

## Navegação Autônoma

### SLAM (Mapeamento e Localização Simultâneos)

O robô de 1ª geração utiliza **GMapping** para criar mapas 2D do ambiente.

#### Características
- **Algoritmo**: Rao-Blackwellized Particle Filter
- **Entrada**: Scans do LIDAR (`/scan`) + Odometria (`/odom`)
- **Saída**: Grade de ocupação 2D (`/map`)
- **Resolução**: 5 cm por célula
- **Partículas**: 30 partículas
- **Taxa de atualização**: 2 Hz

#### Uso
```bash
# Iniciar SLAM
roslaunch robodc_bringup slam.launch

# Teleoperar o robô para mapear
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# Salvar mapa quando terminar
rosrun map_server map_saver -f ~/robodc_ws/maps/nome_do_mapa
```

#### Parâmetros Principais
```yaml
# GMapping configuration
map_update_interval: 2.0    # Frequência de atualização do mapa
maxUrange: 10.0              # Alcance máximo do sensor
particles: 30                # Número de partículas do filtro
linearUpdate: 0.5            # Distância mínima para atualizar (metros)
angularUpdate: 0.5           # Ângulo mínimo para atualizar (radianos)
```

### Localização (AMCL)

Após criar um mapa, o robô usa **AMCL** (Adaptive Monte Carlo Localization) para se localizar.

#### Características
- **Algoritmo**: Monte Carlo Localization com filtro de partículas adaptativo
- **Entrada**: Mapa estático + scans do LIDAR + odometria
- **Saída**: Pose estimada do robô (`/amcl_pose`)
- **Partículas**: 500-2000 (adaptativo)
- **Precisão**: ±5 cm, ±2°

#### Funcionamento
1. **Inicialização**: Define pose inicial com "2D Pose Estimate" no RViz
2. **Predição**: Usa odometria para prever movimento
3. **Correção**: Compara scans com mapa para corrigir pose
4. **Convergência**: Número de partículas reduz quando confiança aumenta

### Planejamento Global

Utiliza o pacote **move_base** com algoritmo **Dijkstra** modificado (NavFn).

#### Características
- **Algoritmo**: NavFn (versão otimizada de Dijkstra)
- **Entrada**: Mapa global + pose inicial + objetivo
- **Saída**: Trajetória global (`/move_base/NavfnROS/plan`)
- **Custo**: Distância ao objetivo + penalidade por obstáculos
- **Tolerância ao objetivo**: 10 cm

#### Comportamento
```python
# O planejador global:
1. Busca caminho de menor custo no mapa global
2. Considera inflação de obstáculos (raio do robô + margem)
3. Replaneja quando obstáculos bloqueiam caminho (1 Hz)
4. Publica trajetória como sequência de waypoints
```

### Planejamento Local (DWA)

**Dynamic Window Approach** - gera trajetórias viáveis em tempo real.

#### Características
- **Janela temporal**: 3 segundos à frente
- **Amostragem**: 
  - Velocidade linear: 20 amostras
  - Velocidade angular: 40 amostras
- **Frequência**: 10 Hz
- **Critérios de avaliação**:
  - Progresso ao objetivo (peso: 24.0)
  - Distância de obstáculos (peso: 32.0)
  - Manutenção de velocidade (peso: 32.0)

#### Parâmetros de Velocidade
```yaml
max_vel_x: 0.5              # m/s
min_vel_x: 0.1              # m/s
max_vel_theta: 1.0          # rad/s
min_vel_theta: -1.0         # rad/s
acc_lim_x: 0.5              # m/s²
acc_lim_theta: 1.0          # rad/s²
```

#### Comportamento de Recovery
Quando o robô fica preso:
1. **Clear costmap**: Limpa obstáculos temporários
2. **Rotate recovery**: Gira 360° para ver ao redor
3. **Back up**: Move para trás se necessário
4. **Abort**: Aborta objetivo se impossível

## Detecção e Desvio de Obstáculos

### Costmap 2D

Sistema de duas camadas para representar obstáculos:

#### Costmap Local (Planejamento Local)
- **Tamanho**: 4m x 4m
- **Resolução**: 2.5 cm
- **Atualização**: 5 Hz
- **Camadas**:
  - **Obstacle Layer**: Obstáculos do LIDAR
  - **Inflation Layer**: Zona de segurança ao redor

#### Costmap Global (Planejamento Global)
- **Tamanho**: Igual ao mapa carregado
- **Resolução**: 5 cm
- **Atualização**: 1 Hz
- **Camadas**:
  - **Static Layer**: Mapa estático
  - **Obstacle Layer**: Obstáculos dinâmicos
  - **Inflation Layer**: Inflação de segurança

### Zona de Segurança
```yaml
robot_radius: 0.20          # Raio do robô (m)
inflation_radius: 0.55      # Raio de inflação (m)
cost_scaling_factor: 10.0   # Fator de escala do custo
```

O robô mantém:
- **0-20 cm**: Zona do robô (custo 254 - letal)
- **20-55 cm**: Zona de inflação (custo 253-1 - decrescente)
- **>55 cm**: Zona livre (custo 0)

## Fusão Sensorial (Robot Localization)

### Extended Kalman Filter (EKF)

Combina múltiplas fontes de dados para odometria precisa.

#### Fontes de Dados
1. **Odometria das rodas** (`/odom_wheels`):
   - Posição (x, y)
   - Velocidade linear
   - Frequência: 50 Hz
   
2. **IMU** (`/imu/data`):
   - Orientação (roll, pitch, yaw)
   - Velocidade angular
   - Aceleração linear
   - Frequência: 100 Hz

#### Configuração do EKF
```yaml
odom0: /odom_wheels
odom0_config: [true,  true,  false,   # x, y, z
               false, false, true,    # roll, pitch, yaw
               true,  true,  false,   # vx, vy, vz
               false, false, true,    # vroll, vpitch, vyaw
               false, false, false]   # ax, ay, az

imu0: /imu/data
imu0_config: [false, false, false,    # x, y, z
              true,  true,  true,     # roll, pitch, yaw
              false, false, false,    # vx, vy, vz
              true,  true,  true,     # vroll, vpitch, vyaw
              false, false, false]    # ax, ay, az
```

#### Resultados
- **Drift de odometria reduzido em ~40%**
- **Estimativa de orientação estável** (IMU compensa derrapagens)
- **Odometria filtrada** publicada em `/odometry/filtered`

## Processamento de Imagem (Básico)

:::note Capacidade Limitada
O robô de 1ª geração tem capacidade limitada de processamento de imagem devido ao hardware (Raspberry Pi 4).
:::

### Funcionalidades Implementadas

#### Detecção de Cor
```python
# Exemplo: Detecção de objeto vermelho
import cv2
import numpy as np

# Converter para HSV
hsv = cv2.cvtColor(frame, HSV)

# Definir range de cor
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])

# Criar máscara
mask = cv2.inRange(hsv, lower_red, upper_red)

# Encontrar contornos
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```

#### Detecção de Linhas (Hough Transform)
Usado para seguir linhas ou detectar corredores:
```python
edges = cv2.Canny(gray, 50, 150)
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
```

#### Detecção de ArUco Markers
Marcadores fiduciais para localização precisa:
```bash
# Launch câmera + detector
roslaunch robodc_sensors aruco_detect.launch
```

**Desempenho**:
- Resolução: 640x480 @ 30 FPS
- Latência: ~100ms
- Alcance de detecção ArUco: 0.5-3 metros

## Teloperação

### Controle por Teclado
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**Teclas**:
- `i`: Frente
- `k`: Parar
- `,`: Ré
- `j`: Girar esquerda
- `l`: Girar direita
- `q/z`: Aumentar/diminuir velocidade máxima

### Controle por Joystick (Opcional)
```bash
roslaunch robodc_control joy_teleop.launch
```

## Limitações Funcionais

1. **Navegação 2D apenas**: Não detecta obstáculos suspensos ou buracos
2. **Ambientes internos**: LIDAR 2D funciona melhor em ambientes estruturados
3. **Velocidade limitada**: 0.5 m/s máximo para navegação segura
4. **Processamento de imagem básico**: Sem deep learning em tempo real
5. **Dependência do roscore**: Sistema para se roscore falhar
[Se implementado, descrever]

### Processamento de LIDAR

#### Filtragem de Dados
- Remoção de ruído
- Downsampling
- Filtro de outliers

#### Extração de Features
- Detecção de linhas
- Identificação de cantos
- Agrupamento de pontos

### Fusão Sensorial

```
Câmera ─────┐
            ├──► Fusão ──► Modelo do Mundo
LIDAR ──────┤
            │
IMU ────────┘
```

- **Método**: Kalman Filter / Particle Filter
- **Estado Estimado**: Pose, velocidade, aceleração
- **Taxa de Atualização**: [Hz]

## Estratégias de Segurança/Fallback

### Níveis de Segurança

#### Nível 1: Operação Normal
- Todos os sistemas funcionando
- Navegação autônoma ativa

#### Nível 2: Modo Degradado
**Condições de Ativação**:
- Perda de sensor secundário
- Bateria em nível de atenção (< 30%)

**Comportamento**:
- Reduzir velocidade máxima
- Aumentar margem de segurança
- Notificar operador

#### Nível 3: Parada de Segurança
**Condições de Ativação**:
- Perda de sensor crítico (LIDAR)
- Bateria crítica (< 10%)
- Erro de comunicação com motor
- Comando de emergência

**Comportamento**:
- Parada controlada imediata
- Desabilitar motores após parada
- Acionar alerta sonoro/visual

### Sistema de Watchdog

```python
class SafetyWatchdog:
    def __init__(self):
        self.last_lidar_time = rospy.Time.now()
        self.last_cmd_time = rospy.Time.now()
        
    def check_sensors(self):
        """Verifica se sensores estão publicando"""
        if (rospy.Time.now() - self.last_lidar_time) > timeout:
            self.trigger_emergency_stop()
            
    def check_communication(self):
        """Verifica comunicação com controladores"""
        # Implementação
        pass
```

### Recuperação de Falhas

#### Falha de Sensor
1. Detectar falha (timeout ou valores inválidos)
2. Isolar sensor com problema
3. Ativar modo degradado
4. Tentar reconexão
5. Se recuperado, retornar à operação normal

#### Falha de Comunicação
1. Detectar perda de comunicação
2. Executar último comando seguro (geralmente parar)
3. Tentar restabelecer comunicação
4. Timeout máximo: [X segundos]

#### Falha de Bateria
1. Monitoramento contínuo de voltagem
2. Aviso em 30%
3. Alerta em 20%
4. Modo seguro em 10%
5. Parada e desligamento em 5%

### Limites Operacionais

#### Limites Físicos
- **Inclinação máxima**: [Graus]
- **Altura máxima de obstáculo**: [cm]
- **Velocidade do vento máxima**: [Se outdoor]

#### Limites Ambientais
- **Temperatura de operação**: [Min - Max °C]
- **Umidade**: [%]
- **Iluminação**: [Lux mínimo para câmera]

### Modo Manual de Emergência

#### Ativação
- Botão físico de emergência
- Comando remoto
- Detecção automática de situação crítica

#### Comportamento
- Desativa navegação autônoma
- Permite controle manual direto
- Mantém sistemas de segurança ativos
- Log de evento

## Monitoramento e Diagnóstico

### Sistema de Logs

```bash
# Logs são salvos em
~/.ros/log/

# Ver logs em tempo real
rostopic echo /diagnostics
```

### Diagnóstico ROS

```python
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# Publicar status de diagnóstico
diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
```

### Métricas Monitoradas

- Taxa de publicação de tópicos
- Uso de CPU e memória
- Temperatura dos componentes
- Estado da bateria
- Latência de comunicação
- Erros de sensores

## Testes de Funcionalidades

### Testes Automatizados

```bash
# Executar suite de testes
rostest robodc_navigation test_navigation.test
```

### Cenários de Teste

1. **Navegação Ponto-a-Ponto**
2. **Desvio de Obstáculos Dinâmicos**
3. **Recuperação de Falhas**
4. **Parada de Emergência**
5. **Operação com Sensores Degradados**

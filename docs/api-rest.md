---
sidebar_position: 7
---

# API REST - RobôDC

## Informações Gerais

- **Repositório**: https://github.com/Hugo-Souza/RoboDC_api
- **Nome**: RoboDC API
- **Versão**: v1.2.3
- **Tecnologias**: Flask, Flask-RESTX, Python 3, ROS 1
- **Licença**: MIT License (Hugo Souza, 2023)

## Visão Geral

A **RoboDC API** é uma API REST desenvolvida em **Flask** que serve como ponte entre aplicações cliente (como o app móvel) e o sistema ROS do robô. Ela expõe funcionalidades de navegação, controle e modificação de expressões faciais através de endpoints HTTP.

:::success API Moderna
Esta API substitui a implementação legada presente em `vivaldini/ROBO_DC/api/`. Oferece melhor arquitetura, documentação Swagger automática e mais funcionalidades.
:::

## Arquitetura

```
┌──────────────────┐
│  Cliente         │ (App Móvel, Web, etc.)
│  HTTP Requests   │
└────────┬─────────┘
         │ REST API (JSON)
         ↓
┌──────────────────┐
│  Flask App       │ (RoboDC_api)
│  Flask-RESTX     │
└────────┬─────────┘
         │ rospy
         ↓
┌──────────────────┐
│  ROS Master      │ (roscore)
│  Topics/Actions  │
└────────┬─────────┘
         │
         ↓
┌──────────────────┐
│  Robô Físico     │
│  Hardware        │
└──────────────────┘
```

## Estrutura do Repositório

```
RoboDC_api/
├── src/
│   ├── __init__.py              # Factory do Flask
│   ├── config.py                # Configurações (dev/test/prod)
│   │
│   └── controllers/             # Controladores da API
│       ├── __init__.py          # Registro de namespaces
│       ├── ros_controller.py    # Endpoints ROS (navegação)
│       ├── led_controller.py    # Expressões faciais (LEDs)
│       └── metadata_controller.py # Metadados da API
│
├── app.py                       # Aplicação principal
├── requirements.txt             # Dependências Python
├── README.md
└── LICENSE.md
```

## Endpoints da API

### Namespace: `/ros`

Endpoints relacionados à navegação e controle ROS.

#### `GET /ros/goal`

Retorna lista de todos os destinos disponíveis no mapa do DC.

**Request**:
```http
GET /ros/goal HTTP/1.1
Host: 192.168.1.100:5000
```

**Response** (200 OK):
```json
{
  "available_locals": [
    "LE-1",
    "LE-2",
    "LE-3",
    "LE-4",
    "Suporte",
    "PPG-CC4",
    "Maker",
    "LE-5",
    "Auditorio",
    "Banheiros",
    "Copa",
    "Lig",
    "Reunioes",
    "Chefia",
    "Graduacao",
    "Recepcao",
    "Home"
  ]
}
```

#### `GET /ros/goTo/{location}`

Envia o robô para um local específico usando `move_base` action.

**Request**:
```http
GET /ros/goTo/Auditorio HTTP/1.1
Host: 192.168.1.100:5000
```

**Response** (200 OK):
```json
{
  "result": "ACTIVE"
}
```

**Possíveis Estados**:
- `PENDING` (0): Aguardando processamento
- `ACTIVE` (1): Robô em movimento
- `SUCCEEDED` (3): Objetivo alcançado
- `ABORTED` (4): Objetivo abortado (obstáculo)
- `REJECTED` (5): Objetivo rejeitado (inválido)
- `PREEMPTED` (2): Preemptado por novo objetivo
- `LOST` (9): Conexão perdida

**Comportamento**:
- Se `{location}` não estiver cadastrado, robô vai para `"Home"` (posição padrão)

#### `GET /ros/status`

Retorna o status atual da navegação.

**Request**:
```http
GET /ros/status HTTP/1.1
Host: 192.168.1.100:5000
```

**Response** (200 OK):
```json
{
  "goal_state": "ACTIVE",
  "comm_state": "ACTIVE"
}
```

**Goal States**:
- `PENDING`, `ACTIVE`, `PREEMPTED`, `SUCCEEDED`, `ABORTED`, `REJECTED`, `PREEMPTING`, `RECALLING`, `RECALLED`, `LOST`

**Comm States**:
- `WAITING_FOR_GOAL_ACK` (0)
- `PENDING` (1)
- `ACTIVE` (2)
- `WAITING_FOR_RESULT` (3)
- `WAITING_FOR_CANCEL_ACK` (4)
- `RECALLING` (5)
- `PREEMPTING` (6)
- `DONE` (7)
- `LOST` (8)

#### `DELETE /ros/cancel`

Cancela todos os objetivos ativos do `move_base`.

**Request**:
```http
DELETE /ros/cancel HTTP/1.1
Host: 192.168.1.100:5000
```

**Response** (200 OK):
```json
{
  "result": "Mensagem de cancelamento enviada."
}
```

### Namespace: `/led`

Endpoints para controle de expressões faciais via Bluetooth (ESP32).

#### Expressões Disponíveis

```python
expressions = {
    # Face completa
    9: "face happy",
    17: "face sad",
    
    # Ambos os olhos
    10: "eyes neutral",
    18: "eyes closed",
    26: "eyes partially_closed",
    34: "eyes partially_open",
    42: "eyes slight_left",
    50: "eyes left",
    58: "eyes realleft",
    66: "eyes slight_right",
    74: "eyes right",
    82: "eyes real_right",
    
    # Olho esquerdo
    11: "left eye neutral",
    19: "left eye closed",
    27: "left eye partially_closed",
    # ... (outros estados)
    
    # Olho direito
    12: "right eye neutral",
    20: "right eye closed",
    28: "right eye partially_closed",
    # ... (outros estados)
    
    # Boca
    13: "mouth happy",
    21: "mouth sad",
    29: "mouth partially_open",
    37: "mouth neutral",
    45: "mouth opened"
}
```

#### `POST /led/changeExpression`

Envia uma sequência de expressões para o ESP32.

**Request**:
```http
POST /led/changeExpression HTTP/1.1
Host: 192.168.1.100:5000
Content-Type: application/json

{
  "expressionValues": [9, 18, 13]
}
```

**Response** (200 OK):
```json
{
  "result": "OK",
  "sentExpressions": [9, 18, 13]
}
```

#### `GET /led/changeExpression/{expressionNumber}`

Muda expressão usando número decimal.

**Request**:
```http
GET /led/changeExpression/9 HTTP/1.1
Host: 192.168.1.100:5000
```

**Response** (200 OK):
```json
{
  "result": "OK",
  "expressionNumber": 9,
  "expressionBits": "0b1001"
}
```

#### `GET /led/changeExpressionByBits/{expressionBits}`

Muda expressão usando representação binária.

**Request**:
```http
GET /led/changeExpressionByBits/1001 HTTP/1.1
Host: 192.168.1.100:5000
```

**Response** (200 OK):
```json
{
  "result": "OK",
  "expressionNumber": 9,
  "expressionBits": "1001"
}
```

#### `GET /led/getExpressionsList`

Retorna lista completa de expressões disponíveis.

**Request**:
```http
GET /led/getExpressionsList HTTP/1.1
Host: 192.168.1.100:5000
```

**Response** (200 OK):
```json
{
  "9": "face happy",
  "17": "face sad",
  "10": "eyes neutral",
  ...
}
```

### Namespace: `/metadata`

Metadados sobre a API.

#### `GET /metadata/version`

Retorna versão atual da API.

**Request**:
```http
GET /metadata/version HTTP/1.1
Host: 192.168.1.100:5000
```

**Response** (200 OK):
```json
{
  "version": "1.2.3"
}
```

## Implementação Interna

### Locais Cadastrados

```python
# src/controllers/ros_controller.py
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
# Formato: (x, y, orientation_z, orientation_w) em metros
```

### Move Base Client

```python
# src/controllers/ros_controller.py
def movebase_client(local):
    global client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Aguarda servidor move_base estar disponível
    client.wait_for_server()
    
    # Cria objetivo
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # Define coordenadas do local
    if local in available_locals:
        goal.target_pose.pose.position.x = available_locals[local][0]
        goal.target_pose.pose.position.y = available_locals[local][1]
        goal.target_pose.pose.orientation.z = available_locals[local][2]
        goal.target_pose.pose.orientation.w = available_locals[local][3]
    else:
        # Fallback para "Home" se local inválido
        goal.target_pose.pose.position.x = available_locals["Home"][0]
        goal.target_pose.pose.position.y = available_locals["Home"][1]
        goal.target_pose.pose.orientation.z = available_locals["Home"][2]
        goal.target_pose.pose.orientation.w = available_locals["Home"][3]
    
    # Envia objetivo
    client.send_goal(goal)
    
    time.sleep(3)  # Aguarda processamento
    
    return client.get_state()
```

### Bluetooth (LEDs)

```python
# src/controllers/led_controller.py
import bluetooth as bt

esp32 = "HEAD"
address = "8C:AA:B5:93:69:EE"  # Endereço MAC do ESP32

def send_expression(expression_value):
    port = 1
    socket = bt.BluetoothSocket(bt.RFCOMM)
    socket.connect((address, port))
    socket.send(bytes([int(expression_value)]))
    socket.close()
```

## Configuração e Deploy

### Instalação

```bash
# Clonar repositório
git clone https://github.com/Hugo-Souza/RoboDC_api.git
cd RoboDC_api

# Criar ambiente virtual
python3 -m venv venv
source venv/bin/activate

# Instalar dependências
pip install -r requirements.txt
```

### Dependências

```
Flask==2.3.0
Flask-RESTX==1.1.0
flask-migrate==4.0.4
flask-sqlalchemy==3.0.5
flask-cors==4.0.0
pybluez==0.23
```

**Dependências ROS**:
```bash
sudo apt install ros-noetic-actionlib
sudo apt install ros-noetic-move-base-msgs
pip install rospkg rospy
```

### Variáveis de Ambiente

```bash
# .env
CONFIG=dev  # ou 'test', 'prod'
ROS_NODE=movebase_client_py
```

### Executar

```bash
# Inicializar ROS
roscore

# Em outro terminal, iniciar API
python app.py

# API estará disponível em:
# http://0.0.0.0:5000
```

### Documentação Swagger

Acesse `http://192.168.1.100:5000/` para visualizar documentação interativa.

## Segurança

:::warning CORS Habilitado
A API tem CORS configurado para aceitar requisições de qualquer origem (`"*"`). Isso é adequado para desenvolvimento e ambientes controlados (rede interna do robô), mas deve ser restrito em produção.
:::

```python
# src/__init__.py
CORS(app, resources={"*": {"origins": "*"}})
```

### Autenticação

A API suporta autenticação via **API Key** (header `Authorization`), mas atualmente não está ativa.

```python
# src/controllers/__init__.py
authorizations = {
    'apikey': {
        'type': 'apiKey',
        'in': 'header',
        'name': 'Authorization'
    }
}
```

## Banco de Dados

Configuração para banco de dados (SQLAlchemy), mas não utilizado ativamente:

```python
# src/config.py
class DevelopmentConfig(Config):
    SQLALCHEMY_DATABASE_URI = 'sqlite:///flask_main.db'

class ProductionConfig(Config):
    SQLALCHEMY_DATABASE_URI = os.getenv('DATABASE_URL')
```

## Troubleshooting

### Problema: `ModuleNotFoundError: No module named 'rospy'`

**Solução**:
```bash
# Certifique-se de que ROS está instalado
source /opt/ros/noetic/setup.bash

# Adicione ao ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

### Problema: `Bluetooth: [Errno 16] Device or resource busy`

**Solução**:
```bash
# Verificar dispositivos Bluetooth conectados
hcitool con

# Desconectar dispositivo
sudo hcitool dc {endereço_MAC}

# Reiniciar serviço Bluetooth
sudo systemctl restart bluetooth
```

### Problema: `SimpleActionClient: move_base not available`

**Solução**:
```bash
# Verificar se move_base está rodando
rosnode list | grep move_base

# Verificar ação disponível
rostopic list | grep move_base

# Iniciar navegação do robô
roslaunch robodc_bringup navigation.launch
```

## Melhorias Futuras

- [ ] Autenticação JWT
- [ ] Rate limiting
- [ ] Logs estruturados
- [ ] Testes unitários e integração
- [ ] Docker container
- [ ] WebSocket para updates em tempo real
- [ ] Suporte a múltiplos robôs
- [ ] Dashboard de monitoramento

## Colaboradores

- **Hugo Souza** (Hugo-Souza) - Desenvolvedor Principal
- **Heitor Souza** (souzaitor) - Contribuidor
- **Thiago Araujo Campos** (thiagoaraujocampos) - Integração App
- **Bruno Leonel** (Bruno12leonel) - Contribuidor

## Links Úteis

- 🔗 [Repositório no GitHub](https://github.com/Hugo-Souza/RoboDC_api)
- 📖 [Flask Documentation](https://flask.palletsprojects.com/)
- 📚 [Flask-RESTX Documentation](https://flask-restx.readthedocs.io/)
- 🤖 [ROS actionlib](http://wiki.ros.org/actionlib)
- 📡 [PyBluez Documentation](https://github.com/pybluez/pybluez)

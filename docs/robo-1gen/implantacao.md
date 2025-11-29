---
sidebar_position: 4
---

# Implantação e Operação

## Visão Geral da Implantação

Este guia cobre a implantação completa do RobôDC de 1ª geração no Departamento de Computação da UFSCar, desde a configuração inicial até operação autônoma em produção.

## Preparação do Ambiente

### 1. Mapeamento do DC (SLAM)

Antes de operar o robô autonomamente, é necessário criar um mapa 2D do ambiente.

#### Equipamentos Necessários
- Robô montado e funcionando
- Laptop/computador com ROS (para RViz e teleoperação)
- Conexão SSH ou rede ROS configurada

#### Procedimento de Mapeamento

```bash
# No Raspberry Pi 4 (robô)
# Terminal 1: Iniciar roscore
roscore

# Terminal 2: Iniciar mobile_rob_dev
rosrun mobile_rob_dev mobile_rob_dev_node

# Terminal 3: Iniciar Hokuyo LiDAR
rosrun urg_node urg_node _ip_address:=192.168.0.10

# Terminal 4: Iniciar GMapping para SLAM
rosrun gmapping slam_gmapping scan:=/scan
```

```bash
# No computador de desenvolvimento
# Configurar variáveis ROS para apontar para o robô
export ROS_MASTER_URI=http://robodc-1gen.local:11311
export ROS_IP=192.168.1.50  # IP do seu computador

# Terminal 1: Visualizar no RViz
rviz

# No RViz, adicionar displays:
# - Map (topic: /map)
# - LaserScan (topic: /scan)
# - RobotModel
# - TF

# Terminal 2: Controlar robô com teclado
sudo apt install ros-noetic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot/cmd_vel
```

#### Dicas para Bom Mapeamento
1. **Velocidade**: Mova o robô LENTAMENTE (~0.2 m/s)
2. **Cobertura**: Passe por todos os corredores, salas e áreas
3. **Sobreposição**: Faça loops fechados (volte ao ponto inicial)
4. **Ângulos**: Gire em diferentes ângulos para melhor cobertura
5. **Tempo**: Leva aproximadamente 30-60 minutos para mapear o DC completo

#### Salvar Mapa

Quando o mapeamento estiver completo:

```bash
# No Raspberry Pi ou computador de desenvolvimento
cd ~/laris_wksp/maps/
rosrun map_server map_saver -f dc_ufscar_map

# Arquivos gerados:
# - dc_ufscar_map.yaml (metadados do mapa)
# - dc_ufscar_map.pgm  (imagem do mapa)
```

Editar `dc_ufscar_map.yaml` se necessário:
```yaml
image: dc_ufscar_map.pgm
resolution: 0.05  # metros por pixel
origin: [-50.0, -50.0, 0.0]  # [x, y, yaw]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### 2. Configurar Localizações dos 17 Locais

Após criar o mapa, anote as coordenadas (x, y) de cada local do DC.

#### Método 1: Via RViz (Recomendado)

```bash
# Carregar mapa no RViz
rosrun map_server map_server dc_ufscar_map.yaml
rviz

# No RViz:
# 1. Adicionar display "Map" (topic: /map)
# 2. Usar tool "Publish Point" para clicar nos locais
# 3. Ver coordenadas no terminal:
rostopic echo /clicked_point

# Anotar coordenadas de cada local
```

#### Método 2: Editar Imagem do Mapa

Usar GIMP ou visualizador de imagens para identificar pixels e converter para metros:
```
x_metros = (pixel_x * resolution) + origin_x
y_metros = (pixel_y * resolution) + origin_y
```

#### Coordenadas dos 17 Locais (Exemplo - AJUSTAR!)

Estas coordenadas devem ser determinadas no seu mapa real:

```python
available_locals = {
    "LE-1": (-37.99, -5.45, 1.0, 0.0),      # Laboratório de Ensino 1
    "LE-2": (-30.15, -5.03, 1.0, 0.0),      # Laboratório de Ensino 2
    "LE-3": (-22.68, -4.45, 1.0, 0.0),      # Laboratório de Ensino 3
    "LE-4": (-15.36, -4.11, 1.0, 0.0),      # Laboratório de Ensino 4
    "LE-5": (9.75, -2.36, 1.0, 0.0),        # Laboratório de Ensino 5
    "Suporte": (-11.30, -3.92, 1.0, 0.0),   # Suporte Técnico
    "PPG-CC4": (-2.54, -3.12, 1.0, 0.0),    # Pós-Graduação CC4
    "Maker": (7.46, -2.39, 1.0, 0.0),       # Espaço Maker
    "Auditorio": (15.37, -1.86, 1.0, 0.0),  # Auditório
    "Banheiros": (-38.74, -10.59, 1.0, 0.0),# Banheiros
    "Copa": (-38.43, -16.47, 1.0, 0.0),     # Copa
    "Lig": (-38.01, -22.61, 1.0, 0.0),      # LIG (Lab. de Inteligência Computacional)
    "Reunioes": (-15.52, -23.80, 1.0, 0.0), # Sala de Reuniões
    "Chefia": (-12.49, -23.54, 1.0, 0.0),   # Chefia do Departamento
    "Graduacao": (-18.67, -24.17, 1.0, 0.0),# Secretaria de Graduação
    "Recepcao": (-12.49, -23.54, 1.0, 0.0), # Recepção
    "Home": (-1.65, -21.18, 1.0, 0.0)       # Posição inicial (Home)
}
```

**Formato**: `(x, y, orientation_z, orientation_w)` em metros e quaternion

#### Atualizar Coordenadas na API

Editar `~/RoboDC_api/src/controllers/ros_controller.py`:

```python
available_locals = {
    "LE-1": (-37.99, -5.45, 1.0, 0.0),
    # ... suas coordenadas reais aqui
}
```

## Configuração da API REST

### 1. Instalar e Configurar API

```bash
cd ~
git clone https://github.com/Hugo-Souza/RoboDC_api.git
cd RoboDC_api

# Criar ambiente virtual
python3 -m venv venv
source venv/bin/activate

# Instalar dependências
pip install -r requirements.txt
```

### 2. Configurar Bluetooth (ESP32 para LEDs)

```bash
# Instalar bluez
sudo apt install bluetooth bluez libbluetooth-dev

# Emparelhar ESP32
bluetoothctl
scan on
# Aguardar detectar: 8C:AA:B5:93:69:EE
pair 8C:AA:B5:93:69:EE
trust 8C:AA:B5:93:69:EE
exit
```

### 3. Testar API Manualmente

```bash
# Terminal 1: roscore
roscore

# Terminal 2: mobile_rob_dev
rosrun mobile_rob_dev mobile_rob_dev_node

# Terminal 3: move_base (navegação)
rosrun move_base move_base

# Terminal 4: API
cd ~/RoboDC_api
source venv/bin/activate
python3 app.py

# Testar endpoints
curl http://localhost:5000/metadata/version
curl http://localhost:5000/ros/available_locals
curl http://localhost:5000/ros/goTo/LE-1
```

## Configuração do Aplicativo Móvel (Tablet)

### 1. Instalar APK no Tablet Android

```bash
# No computador de desenvolvimento
cd ~
git clone https://github.com/thiagoaraujocampos/RoboDC.git
cd RoboDC
npm install
ionic capacitor build android

# Gerar APK
cd android
./gradlew assembleDebug
# APK em: android/app/build/outputs/apk/debug/app-debug.apk

# Transferir para tablet via USB ou Google Drive
# Instalar APK no tablet
```

### 2. Configurar Endereço da API no App

Editar `src/environments/environment.ts`:

```typescript
export const environment = {
  production: false,
  apiUrl: 'http://192.168.1.100:5000'  // IP do Raspberry Pi 4
};
```

Rebuildar e reinstalar APK.

### 3. Conectar Tablet ao Wi-Fi MrRoboto

No tablet:
1. Configurações → Wi-Fi
2. Conectar à rede "MrRoboto"
3. Senha: [sua_senha]

### 4. Emparelhar Bluetooth (ESP32)

No aplicativo:
1. Abrir aba "Expressões"
2. Botão "Conectar Bluetooth"
3. Selecionar dispositivo ESP32 (`8C:AA:B5:93:69:EE`)
4. Testar expressões faciais

## Inicialização Automática (Systemd)

### 1. Criar Serviço para ROS + mobile_rob_dev

```bash
sudo nano /etc/systemd/system/robodc-ros.service
```

Conteúdo:
```ini
[Unit]
Description=RoboDC ROS Core and mobile_rob_dev
After=network.target

[Service]
Type=forking
User=ubuntu
Environment="ROS_MASTER_URI=http://localhost:11311"
Environment="ROS_IP=192.168.1.100"

# Iniciar roscore em background
ExecStartPre=/bin/bash -c 'source /opt/ros/noetic/setup.bash && roscore &'
ExecStartPre=/bin/sleep 5

# Iniciar mobile_rob_dev
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source /home/ubuntu/laris_wksp/devel/setup.bash && rosrun mobile_rob_dev mobile_rob_dev_node &'

ExecStop=/usr/bin/killall -SIGINT roslaunch rosrun
ExecStop=/bin/sleep 2
ExecStop=/usr/bin/killall -9 roscore rosmaster

Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### 2. Criar Serviço para Hokuyo LiDAR

```bash
sudo nano /etc/systemd/system/robodc-lidar.service
```

Conteúdo:
```ini
[Unit]
Description=RoboDC Hokuyo LiDAR
After=robodc-ros.service
Requires=robodc-ros.service

[Service]
Type=simple
User=ubuntu
Environment="ROS_MASTER_URI=http://localhost:11311"
Environment="ROS_IP=192.168.1.100"

ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosrun urg_node urg_node _ip_address:=192.168.0.10'

Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### 3. Criar Serviço para Navegação (move_base + AMCL)

```bash
sudo nano /etc/systemd/system/robodc-navigation.service
```

Conteúdo:
```ini
[Unit]
Description=RoboDC Navigation Stack
After=robodc-lidar.service
Requires=robodc-lidar.service

[Service]
Type=simple
User=ubuntu
Environment="ROS_MASTER_URI=http://localhost:11311"
Environment="ROS_IP=192.168.1.100"

# Carregar mapa e iniciar AMCL + move_base
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source /home/ubuntu/laris_wksp/devel/setup.bash && roslaunch [seu_package] navigation.launch map_file:=/home/ubuntu/laris_wksp/maps/dc_ufscar_map.yaml'

Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### 4. Criar Serviço para API REST

```bash
sudo nano /etc/systemd/system/robodc-api.service
```

Conteúdo:
```ini
[Unit]
Description=RoboDC REST API (Flask)
After=robodc-navigation.service
Requires=robodc-navigation.service

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/home/ubuntu/RoboDC_api

ExecStart=/home/ubuntu/RoboDC_api/venv/bin/python3 /home/ubuntu/RoboDC_api/app.py

Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### 5. Habilitar e Iniciar Serviços

```bash
# Recarregar systemd
sudo systemctl daemon-reload

# Habilitar serviços (iniciar no boot)
sudo systemctl enable robodc-ros.service
sudo systemctl enable robodc-lidar.service
sudo systemctl enable robodc-navigation.service
sudo systemctl enable robodc-api.service

# Iniciar serviços manualmente
sudo systemctl start robodc-ros.service
sudo systemctl start robodc-lidar.service
sudo systemctl start robodc-navigation.service
sudo systemctl start robodc-api.service

# Verificar status
sudo systemctl status robodc-ros.service
sudo systemctl status robodc-lidar.service
sudo systemctl status robodc-navigation.service
sudo systemctl status robodc-api.service

# Ver logs
journalctl -u robodc-ros.service -f
```

## Operação em Produção

### Inicialização do Sistema

1. **Ligar baterias**:
   - Bateria 1 (LiDAR) - interruptor 1
   - Bateria 2 (sistema geral) - interruptor 2

2. **Aguardar boot** (30-60 segundos)
   - Raspberry Pi 4 inicia Ubuntu
   - Serviços systemd são iniciados automaticamente

3. **Verificar LEDs de status**:
   - Raspberry Pi: LED verde piscando (atividade)
   - LiDAR: LED girando (escaneando)
   - ESP32: LEDs acesos (face padrão)

4. **Conectar tablet**:
   - Abrir aplicativo RobôDC
   - Conectar Wi-Fi MrRoboto
   - App deve mostrar status "Conectado"

### Enviar Robô para um Local

1. Abrir aba "Navegação" no app
2. Selecionar local no mapa (ex: "Auditório")
3. Confirmar "Guiar-me até lá"
4. App mostra status "ACTIVE" (em movimento)
5. Robô navega autonomamente
6. Ao chegar, status muda para "SUCCEEDED"
7. Robô anuncia via TTS: "Cheguei ao Auditório"

### Mudar Expressão Facial

1. Abrir aba "Expressões"
2. Selecionar expressão (ex: "Feliz")
3. Enviar comando via Bluetooth
4. LEDs mudam imediatamente

### Controle Manual (Emergência)

1. Abrir aba "Controle Manual"
2. Usar joystick virtual para mover robô
3. Comandos são enviados diretamente para `/robot/cmd_vel`

### Parada de Emergência

- **Botão de Emergência Físico**: Pressionar botão vermelho no robô
- **No App**: Aba "Navegação" → Botão "Cancelar"
- **SSH**: `rostopic pub /robot/cmd_vel geometry_msgs/Twist` (velocidade zero)

## Monitoramento e Diagnóstico

### Monitoramento Remoto via SSH

```bash
# Conectar via SSH
ssh ubuntu@robodc-1gen.local

# Verificar serviços
sudo systemctl status robodc-*.service

# Ver nós ROS ativos
rosnode list

# Ver tópicos
rostopic list

# Verificar frequência de sensores
rostopic hz /scan       # Deve ser ~10 Hz
rostopic hz /odom       # Deve ser ~40 Hz

# Ver pose atual
rostopic echo /amcl_pose -n 1
```

### Dashboard de Monitoramento (Opcional)

Usar `rqt` para interface gráfica:

```bash
# No computador de desenvolvimento
export ROS_MASTER_URI=http://robodc-1gen.local:11311
export ROS_IP=192.168.1.50

rqt
```

Plugins úteis:
- **Topic Monitor**: Ver taxa de publicação
- **Message Publisher**: Publicar mensagens manualmente
- **Service Caller**: Chamar serviços
- **TF Tree**: Visualizar árvore de transformadas

### Logs e Troubleshooting

```bash
# Logs do ROS
tail -f ~/.ros/log/latest/rosout.log

# Logs dos serviços systemd
journalctl -u robodc-ros.service -n 100
journalctl -u robodc-api.service -f

# Verificar erros ROS
rosrun rqt_console rqt_console

# Diagnóstico geral
roswtf
```

## Manutenção Regular

### Diária
- ✅ Verificar nível de bateria
- ✅ Limpar janela do LiDAR
- ✅ Verificar logs de erro

### Semanal
- ✅ Verificar conexões físicas (cabos USB, Ethernet)
- ✅ Limpar pó da estrutura
- ✅ Testar navegação para todos os 17 locais
- ✅ Verificar todas as 45 expressões faciais

### Mensal
- ✅ Atualizar software (`git pull`, `catkin build`)
- ✅ Atualizar sistema operacional (`sudo apt upgrade`)
- ✅ Backup do mapa e configurações
- ✅ Verificar tensão das baterias VRLA (deve ser ~12V)
- ✅ Limpar logs antigos (`rosclean purge`)

### Trimestral
- ✅ Recalibrar odometria (se necessário)
- ✅ Atualizar mapa (se houve mudanças no DC)
- ✅ Revisar parâmetros de navegação (DWA, costmaps)

## Troubleshooting em Produção

### Robô não liga
- Verificar interruptores das baterias
- Medir tensão das baterias (deve ser >11V)
- Verificar fusíveis
- Verificar conexões de alimentação

### LiDAR não publica dados
- Verificar cabo Ethernet conectado
- Verificar IP: `ping 192.168.0.10`
- Verificar se bateria dedicada do LiDAR está ligada
- Reiniciar serviço: `sudo systemctl restart robodc-lidar.service`

### Robô não navega (fica parado)
- Verificar se Raspberry Pi Pico responde
- Testar comando manual: `rostopic pub /robot/cmd_vel ...`
- Verificar conexão serial: `ls -l /dev/ttyACM0`
- Verificar logs do mobile_rob_dev

### API não responde
- Verificar se Flask está rodando: `ps aux | grep flask`
- Testar localmente: `curl http://localhost:5000/metadata/version`
- Verificar firewall: `sudo ufw status`
- Reiniciar serviço: `sudo systemctl restart robodc-api.service`

### Aplicativo não conecta
- Verificar se tablet está em rede MrRoboto
- Verificar IP da API no app (192.168.1.100)
- Testar ping: `ping 192.168.1.100`
- Verificar se API está rodando

### Odometria deriva muito
- Recalibrar parâmetros `wheel_radius` e `wheel_base`
- Verificar aderência das rodas
- Verificar piso (superfície escorregadia?)
- Reinicializar AMCL no RViz ("2D Pose Estimate")

## Backup e Recuperação

### Criar Backup do Sistema

```bash
# Backup do workspace ROS
cd ~
tar -czf robodc-workspace-$(date +%Y%m%d).tar.gz laris_wksp/

# Backup da API
tar -czf robodc-api-$(date +%Y%m%d).tar.gz RoboDC_api/

# Backup de configurações
sudo tar -czf robodc-config-$(date +%Y%m%d).tar.gz \
  /etc/systemd/system/robodc-*.service \
  /etc/udev/rules.d/99-robodc.rules \
  /etc/netplan/

# Backup do mapa
tar -czf robodc-maps-$(date +%Y%m%d).tar.gz laris_wksp/maps/

# Copiar backups para servidor remoto
scp robodc-*.tar.gz usuario@servidor:/backups/robodc/
```

### Restaurar do Backup

```bash
# Restaurar workspace
cd ~
tar -xzf robodc-workspace-20251129.tar.gz

# Recompilar
cd laris_wksp
catkin build
source devel/setup.bash

# Restaurar API
cd ~
tar -xzf robodc-api-20251129.tar.gz
cd RoboDC_api
pip3 install -r requirements.txt

# Restaurar configurações
sudo tar -xzf robodc-config-20251129.tar.gz -C /

# Recarregar systemd
sudo systemctl daemon-reload
```

## Descomissionamento Seguro

Quando precisar desligar o robô:

```bash
# 1. Parar serviços ROS
sudo systemctl stop robodc-api.service
sudo systemctl stop robodc-navigation.service
sudo systemctl stop robodc-lidar.service
sudo systemctl stop robodc-ros.service

# 2. Desligar Raspberry Pi
sudo shutdown -h now

# 3. Aguardar 30 segundos

# 4. Desligar baterias (interruptores)
```

---

**Próximo**: [Software - Arquitetura ROS1](./software/arquitetura-ros1.md)

**Ver também**: [Hardware](./hardware.md) | [Instalação](./instalacao.md)


```bash
# Drivers de câmera
sudo apt install ros-noetic-usb-cam

# Drivers de LIDAR
sudo apt install ros-noetic-urg-node

# Outras dependências específicas
cd ~/catkin_ws/src/robodc-1gen
./scripts/install_hardware_deps.sh
```

### Criar Imagem do Sistema (Opcional)

#### Usando CloneZilla ou dd

```bash
# Criar backup do disco
sudo dd if=/dev/sda of=robodc-1gen-backup.img bs=4M status=progress

# Comprimir
gzip robodc-1gen-backup.img
```

#### Criar Snapshot Docker (Alternativa)

[Se o sistema usar containers]

```bash
docker commit robodc-container robodc-1gen:latest
docker save robodc-1gen:latest | gzip > robodc-1gen-docker.tar.gz
```

## Configurações em Tempo de Execução

### Configurações de Rede

#### Configuração Estática (Recomendado)

```bash
# Editar /etc/netplan/01-network-manager-all.yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]

# Aplicar
sudo netplan apply
```

#### Configuração WiFi (Se aplicável)

```yaml
wifis:
  wlan0:
    dhcp4: no
    addresses: [192.168.1.101/24]
    gateway4: 192.168.1.1
    access-points:
      "NOME_DA_REDE":
        password: "senha"
```

### Configuração de Inicialização Automática

#### Criar Serviço Systemd

```bash
# Criar arquivo /etc/systemd/system/robodc.service
sudo nano /etc/systemd/system/robodc.service
```

```ini
[Unit]
Description=RoboDC 1st Generation Robot Service
After=network.target

[Service]
Type=simple
User=robot
Environment="ROS_MASTER_URI=http://localhost:11311"
Environment="ROS_HOSTNAME=robodc"
ExecStart=/home/robot/catkin_ws/src/robodc-1gen/scripts/start_robot.sh
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

```bash
# Habilitar serviço
sudo systemctl enable robodc.service

# Iniciar serviço
sudo systemctl start robodc.service

# Verificar status
sudo systemctl status robodc.service
```

#### Script de Inicialização

```bash
#!/bin/bash
# ~/catkin_ws/src/robodc-1gen/scripts/start_robot.sh

# Source ROS
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Aguardar rede estar disponível
sleep 10

# Iniciar roscore em background
roscore &
ROSCORE_PID=$!
sleep 5

# Lançar o robô
roslaunch robodc_1gen robot.launch

# Cleanup ao sair
kill $ROSCORE_PID
```

### Parâmetros de Calibração

#### Calibração de Sensores

```yaml
# config/sensor_calibration.yaml
camera:
  calibration_file: "$(find robodc_1gen)/calibration/camera_calibration.yaml"
  
lidar:
  angle_min: -3.14159
  angle_max: 3.14159
  offset_x: 0.15
  offset_y: 0.0
  offset_z: 0.3

imu:
  orientation_covariance: [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
  angular_velocity_covariance: [0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02]
```

#### Parâmetros do Robô

```yaml
# config/robot_params.yaml
robot:
  wheel_base: 0.3          # metros
  wheel_radius: 0.075      # metros
  max_linear_velocity: 1.0  # m/s
  max_angular_velocity: 2.0 # rad/s
  
control:
  pid:
    kp: 1.0
    ki: 0.1
    kd: 0.05
```

### Logs e Monitoramento

#### Configurar ROS Logging

```bash
# ~/.ros/config/rosconsole.config
log4j.logger.ros=INFO
log4j.logger.ros.robodc_navigation=DEBUG
log4j.logger.ros.robodc_control=INFO
```

#### Sistema de Logs Persistente

```bash
# Criar script de rotação de logs
# /etc/logrotate.d/robodc
/home/robot/.ros/log/*.log {
    daily
    rotate 7
    compress
    missingok
    notifempty
}
```

## Dicas de Troubleshooting

### Problemas Comuns de Implantação

#### 1. ROS Master não Inicia

**Sintomas**: Erro ao conectar ao ROS Master

**Diagnóstico**:
```bash
# Verificar se roscore está rodando
ps aux | grep roscore

# Verificar portas
netstat -tulpn | grep 11311
```

**Soluções**:
```bash
# Matar processos ROS antigos
killall -9 roscore rosmaster

# Limpar arquivos temporários
rm -rf ~/.ros/log/*

# Reiniciar roscore
roscore
```

#### 2. Sensores não Conectam

**Sintomas**: Nós de sensores não publicam dados

**Diagnóstico**:
```bash
# Listar dispositivos USB
lsusb

# Verificar permissões
ls -l /dev/ttyUSB* /dev/video*

# Ver logs do ROS
rostopic echo /rosout
```

**Soluções**:
```bash
# Adicionar usuário aos grupos corretos
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER

# Configurar regras udev
sudo nano /etc/udev/rules.d/99-robodc.rules
```

```
# Regras udev exemplo
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout"
SUBSYSTEM=="video4linux", MODE="0666", GROUP="video"
```

```bash
# Recarregar regras
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 3. Transformações (TF) Faltando

**Sintomas**: Erro "Could not find a connection between frames"

**Diagnóstico**:
```bash
# Visualizar árvore TF
rosrun tf view_frames
evince frames.pdf

# Verificar TF em tempo real
rosrun tf tf_echo base_link camera_link
```

**Soluções**:
- Verificar se `robot_state_publisher` está rodando
- Confirmar que URDF está carregado corretamente
- Verificar launch files

#### 4. Alto Uso de CPU/Memória

**Diagnóstico**:
```bash
# Monitorar recursos
htop

# Perfil do ROS
rostopic hz /topic_name
rostopic bw /topic_name
```

**Soluções**:
- Reduzir taxa de publicação de tópicos
- Diminuir resolução de imagens
- Otimizar algoritmos
- Aumentar hardware se necessário

#### 5. Bateria Descarrega Rápido

**Diagnóstico**:
```bash
# Monitorar consumo
rostopic echo /battery_state

# Verificar processos que consomem energia
sudo powertop
```

**Soluções**:
- Desabilitar features não essenciais
- Reduzir brilho de displays
- Otimizar uso de WiFi
- Verificar curto-circuito em hardware

### Ferramentas de Diagnóstico

```bash
# Verificar status geral
roswtf

# Diagnóstico de rede ROS
rosrun rqt_graph rqt_graph

# Monitor de tópicos
rosrun rqt_topic rqt_topic

# Console de mensagens
rosrun rqt_console rqt_console
```

### Checklist de Implantação

- [ ] Sistema operacional instalado e atualizado
- [ ] ROS instalado e configurado
- [ ] Código do robô clonado e compilado
- [ ] Dependências instaladas
- [ ] Drivers de hardware configurados
- [ ] Permissões de dispositivos configuradas
- [ ] Rede configurada (IP estático)
- [ ] Serviço de inicialização automática criado
- [ ] Parâmetros de calibração ajustados
- [ ] Logs configurados
- [ ] Testes básicos executados
- [ ] Documentação de configuração salva
- [ ] Backup do sistema criado

### Manutenção Regular

#### Semanal
- Verificar logs de erro
- Testar sensores
- Verificar nível de bateria

#### Mensal
- Atualizar sistema operacional
- Limpar logs antigos
- Calibrar sensores
- Backup incremental

#### Trimestral
- Atualização de software
- Revisão de hardware
- Backup completo do sistema

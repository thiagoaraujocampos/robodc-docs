---
sidebar_position: 4
---

# Implantação - 1ª Geração

## Preparação do Sistema Base

### 1. Instalação do Ubuntu 20.04

#### Download e Criação de Mídia Bootável
```bash
# Para Raspberry Pi 4
# Baixar: Ubuntu 20.04 LTS Server ARM64 de ubuntu.com/download/raspberry-pi
# Usar Raspberry Pi Imager ou Balena Etcher para gravar no SD Card (mínimo 32GB)

# Para Intel NUC
# Baixar: Ubuntu 20.04 LTS Desktop AMD64
# Criar pendrive bootável com Rufus (Windows) ou dd (Linux)
```

#### Primeira Inicialização
```bash
# Conectar monitor, teclado e ethernet
# Login padrão (Raspberry Pi): ubuntu/ubuntu (será solicitado trocar senha)

# Expandir filesystem (Raspberry Pi)
sudo apt update
sudo apt install raspi-config
sudo raspi-config
# Advanced Options → Expand Filesystem

# Configurar hostname
sudo hostnamectl set-hostname robodc-1gen

# Atualizar sistema
sudo apt update
sudo apt upgrade -y
```

### 2. Instalação do ROS Noetic

```bash
# Configurar sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Adicionar chave do repositório
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Atualizar índice de pacotes
sudo apt update

# Instalar ROS Noetic Desktop Full (para desenvolvimento)
# OU ROS Base (para produção, economiza espaço)
sudo apt install ros-noetic-desktop-full  # ~2.5GB
# sudo apt install ros-noetic-ros-base    # ~300MB (produção)

# Configurar environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Instalar ferramentas de build
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools

# Inicializar rosdep
sudo rosdep init
rosdep update
```

### 3. Configuração de Rede

#### Para Operação Standalone
```bash
# Editar ~/.bashrc
nano ~/.bashrc

# Adicionar ao final:
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_HOSTNAME=$(hostname)
```

#### Para Operação com Computador Remoto
```bash
# No robô
export ROS_MASTER_URI=http://robodc-1gen.local:11311  # ou IP do robô
export ROS_IP=$(hostname -I | awk '{print $1}')

# No computador de desenvolvimento
export ROS_MASTER_URI=http://robodc-1gen.local:11311
export ROS_IP=$(hostname -I | awk '{print $1}')
```

### 4. Configuração de Permissões e Dispositivos

```bash
# Adicionar usuário aos grupos necessários
sudo usermod -aG dialout $USER
sudo usermod -aG video $USER
sudo usermod -aG gpio $USER  # Apenas Raspberry Pi

# Criar regras udev
sudo nano /etc/udev/rules.d/99-robodc.rules
```

Conteúdo do arquivo:
```
# RPLidar
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"

# Arduino (se usado para controle de motores)
KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", MODE:="0666", GROUP:="dialout", SYMLINK+="arduino"

# Câmera USB
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="082d", MODE:="0666", GROUP:="video", SYMLINK+="camera"
```

Aplicar regras:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Instalação do Software RobôDC

### 1. Clonar Repositórios

```bash
# Criar workspace
mkdir -p ~/robodc_ws/src
cd ~/robodc_ws/src

# Clonar pacotes
git clone https://github.com/robodc/robodc_bringup.git
git clone https://github.com/robodc/robodc_description.git
git clone https://github.com/robodc/robodc_control.git
git clone https://github.com/robodc/robodc_sensors.git
git clone https://github.com/robodc/robodc_navigation.git
git clone https://github.com/robodc/robodc_msgs.git

# OU clonar repositório monorepo
# git clone --recursive https://github.com/robodc/robodc_1gen.git
```

### 2. Instalar Dependências

```bash
cd ~/robodc_ws

# Instalar dependências ROS
rosdep install --from-paths src --ignore-src -r -y

# Instalar dependências Python adicionais (se necessário)
pip3 install numpy scipy matplotlib
```

### 3. Compilar

```bash
cd ~/robodc_ws

# Compilar com otimização Release
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

# OU com catkin_make
# catkin_make -DCMAKE_BUILD_TYPE=Release

# Source do workspace
source devel/setup.bash
echo "source ~/robodc_ws/devel/setup.bash" >> ~/.bashrc
```

### 4. Verificar Instalação

```bash
# Verificar pacotes
rospack list | grep robodc

# Testar sensores individualmente
roslaunch robodc_sensors rplidar.launch  # Deve conectar ao LIDAR
roslaunch robodc_sensors camera.launch   # Deve abrir câmera
roslaunch robodc_sensors imu.launch      # Deve publicar /imu/data
```

## Configuração para Inicialização Automática

### Criar Serviço Systemd

```bash
sudo nano /etc/systemd/system/robodc.service
```

Conteúdo:
```ini
[Unit]
Description=RoboDC 1st Generation Robot System
After=network.target

[Service]
Type=simple
User=ubuntu
Group=ubuntu
WorkingDirectory=/home/ubuntu/robodc_ws

# Iniciar roscore e aguardar
ExecStartPre=/bin/bash -c 'source /opt/ros/noetic/setup.bash && roscore &'
ExecStartPre=/bin/sleep 5

# Iniciar sistema do robô
ExecStart=/bin/bash -c 'source /opt/ros/noetic/setup.bash && source /home/ubuntu/robodc_ws/devel/setup.bash && roslaunch robodc_bringup robot.launch'

# Desligar graciosamente
ExecStop=/usr/bin/killall -SIGINT roslaunch
ExecStop=/bin/sleep 2
ExecStop=/usr/bin/killall -9 rosmaster

Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Habilitar serviço:
```bash
sudo systemctl daemon-reload
sudo systemctl enable robodc.service
sudo systemctl start robodc.service

# Verificar status
sudo systemctl status robodc.service

# Ver logs
journalctl -u robodc.service -f
```

### Script de Inicialização Alternativo (sem systemd)

```bash
# Criar script
nano ~/start_robodc.sh
```

Conteúdo:
```bash
#!/bin/bash

# Source ROS
source /opt/ros/noetic/setup.bash
source ~/robodc_ws/devel/setup.bash

# Esperar rede estar pronta
sleep 10

# Iniciar roscore em background
roscore &
sleep 5

# Iniciar sistema do robô
roslaunch robodc_bringup robot.launch
```

Tornar executável e adicionar ao cron:
```bash
chmod +x ~/start_robodc.sh

# Adicionar ao crontab para iniciar no boot
crontab -e
# Adicionar linha:
@reboot /home/ubuntu/start_robodc.sh >> /home/ubuntu/robodc.log 2>&1
```

## Criação de Imagem do Sistema

### Backup Completo (Raspberry Pi)

```bash
# No computador de desenvolvimento (Linux)
# Com SD card do Raspberry conectado via leitor USB

# Identificar dispositivo
lsblk  # Ex: /dev/sdb

# Criar imagem
sudo dd if=/dev/sdb of=~/robodc-1gen-image.img bs=4M status=progress

# Comprimir imagem
gzip -9 ~/robodc-1gen-image.img
# Resultado: robodc-1gen-image.img.gz (~4-8GB dependendo do cartão)

# Para restaurar em outro SD card:
gunzip robodc-1gen-image.img.gz
sudo dd if=robodc-1gen-image.img of=/dev/sdb bs=4M status=progress
```

### Backup do Workspace (Intel NUC ou multiplataforma)

```bash
# No robô, criar tarball do workspace
cd ~
tar -czf robodc-workspace-backup.tar.gz robodc_ws/

# Incluir configurações
tar -czf robodc-config-backup.tar.gz .bashrc /etc/udev/rules.d/99-robodc.rules /etc/systemd/system/robodc.service

# Copiar para computador remoto
scp robodc-*.tar.gz usuario@computador-remoto:~/backups/
```

## Procedimentos de Manutenção

### Atualização de Software

```bash
# Parar sistema
sudo systemctl stop robodc.service

# Atualizar código
cd ~/robodc_ws/src/robodc_bringup  # ou outro pacote
git pull origin main

# Recompilar apenas pacotes modificados
cd ~/robodc_ws
catkin build robodc_bringup  # apenas o pacote alterado

# Reiniciar sistema
sudo systemctl start robodc.service
```

### Verificação de Saúde do Sistema

```bash
# Verificar uso de disco
df -h

# Verificar memória
free -h

# Verificar temperatura (Raspberry Pi)
vcgencmd measure_temp

# Verificar processos ROS
rosnode list
rostopic hz /scan  # verificar frequência de publicação

# Verificar logs
tail -f ~/.ros/log/latest/roslaunch-*.log
```

### Limpeza de Logs

```bash
# ROS gera muitos logs, limpar periodicamente
rosclean check  # Ver quanto espaço os logs ocupam
rosclean purge  # Deletar todos os logs antigos

# Limpar logs do sistema
sudo journalctl --vacuum-time=7d  # Manter apenas últimos 7 dias
```

## Troubleshooting Pós-Implantação

### Robô não Inicia Automaticamente
```bash
# Verificar status do serviço
sudo systemctl status robodc.service

# Ver erros
journalctl -u robodc.service -n 50

# Verificar se roscore está rodando
pgrep -a roscore

# Testar manualmente
source /opt/ros/noetic/setup.bash
source ~/robodc_ws/devel/setup.bash
roslaunch robodc_bringup robot.launch
```

### Sensores não Conectam
```bash
# Verificar dispositivos USB
lsusb
ls -l /dev/ttyUSB* /dev/video*

# Verificar permissões
groups  # Deve incluir dialout, video

# Recarregar regras udev
sudo udevadm control --reload-rules
sudo udevadm trigger

# Reconectar dispositivos USB
```

### Performance Baixa
```bash
# Verificar CPU/memória
top
htop  # se instalado

# Para Raspberry Pi: Reduzir carga
# - Desabilitar RVIZ/GUI no launch file
# - Reduzir resolução da câmera
# - Reduzir frequência de publicação dos sensores

# Exemplo: Editar launch file para não usar GUI
<arg name="gui" default="false"/>
```

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

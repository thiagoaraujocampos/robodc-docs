---
sidebar_position: 2
---

# Instalação e Configuração

## Pré-requisitos

### Sistema Operacional

**Ubuntu 20.04 LTS (Focal Fossa)** - obrigatório para ROS 1 Noetic

- Arquitetura: ARM64 para Raspberry Pi 4
- Instalação: Ubuntu Server 20.04 LTS (mínimo 32GB microSD, recomendado 64GB)

:::warning Compatibilidade
ROS 1 Noetic **não** é compatível com Ubuntu 22.04 ou versões superiores. Use exclusivamente Ubuntu 20.04 LTS.
:::

### Hardware Necessário

- **Raspberry Pi 4 Model B** (4GB RAM)
- MicroSD Card 64GB (Classe 10 ou UHS-I)
- Fonte de alimentação USB-C 5V/3A
- Cabo Ethernet (para configuração inicial)
- Teclado e monitor HDMI (para configuração inicial)

## Instalação do Ubuntu 20.04 no Raspberry Pi 4

### 1. Download e Preparação do SD Card

```bash
# Baixar Ubuntu Server 20.04 LTS ARM64
# URL: https://ubuntu.com/download/raspberry-pi

# Usar Raspberry Pi Imager (recomendado)
# ou Balena Etcher para gravar a imagem no SD Card
```

### 2. Primeira Inicialização

```bash
# Inserir SD Card no Raspberry Pi 4
# Conectar cabo Ethernet, teclado, monitor HDMI
# Conectar fonte de alimentação

# Login padrão
# Usuário: ubuntu
# Senha: ubuntu
# (Será solicitado trocar senha na primeira vez)

# Expandir filesystem
sudo apt update
sudo apt install raspi-config -y
sudo raspi-config
# Advanced Options → Expand Filesystem → Reboot

# Configurar hostname
sudo hostnamectl set-hostname robodc-1gen

# Atualizar sistema
sudo apt update
sudo apt upgrade -y
```

### 3. Configurar Rede Wi-Fi (Opcional)

```bash
# Editar configuração de rede
sudo nano /etc/netplan/50-cloud-init.yaml
```

Adicione:
```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "MrRoboto":
          password: "sua_senha_aqui"
```

Aplicar configuração:
```bash
sudo netplan apply
```

## Instalação do ROS 1 Noetic

### 1. Configurar Repositórios

```bash
# Adicionar repositório do ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Adicionar chave GPG
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Atualizar índice de pacotes
sudo apt update
```

### 2. Instalar ROS Noetic

```bash
# Instalação base (recomendado para Raspberry Pi - economiza espaço)
sudo apt install ros-noetic-ros-base -y

# OU Desktop (inclui RViz, rqt - usa mais espaço)
# sudo apt install ros-noetic-desktop -y

# Configurar environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Instalar ferramentas de build
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools -y

# Inicializar rosdep
sudo rosdep init
rosdep update
```

### 3. Instalar Pacotes ROS Necessários

```bash
# Navegação
sudo apt install ros-noetic-navigation ros-noetic-move-base ros-noetic-amcl ros-noetic-map-server -y

# Drivers de sensores
sudo apt install ros-noetic-urg-node -y  # Hokuyo LiDAR

# Ferramentas de desenvolvimento
sudo apt install ros-noetic-tf ros-noetic-tf2-tools ros-noetic-robot-state-publisher -y
```

## Clonar e Compilar o Repositório

### 1. Criar Workspace

```bash
# Criar workspace
mkdir -p ~/laris_wksp/src
cd ~/laris_wksp/src
```

### 2. Clonar Repositório vivaldini/ROBO_DC

```bash
# Clonar repositório principal
git clone https://github.com/vivaldini/ROBO_DC.git

# Verificar estrutura
cd ROBO_DC
ls -la
# Deve mostrar: mobile_rob_dev, mobile_rob_dev_sim, envrobotz, api, etc.
```

### 3. Instalar Dependências

```bash
cd ~/laris_wksp

# Instalar dependências do ROS
rosdep install --from-paths src --ignore-src -r -y

# Dependências adicionais (se necessário)
sudo apt install python3-serial python3-pip -y
pip3 install pyserial
```

### 4. Compilar

```bash
cd ~/laris_wksp

# Compilar com catkin build (recomendado)
catkin build -DCMAKE_BUILD_TYPE=Release

# OU com catkin_make
# catkin_make -DCMAKE_BUILD_TYPE=Release

# Source do workspace
source devel/setup.bash
echo "source ~/laris_wksp/devel/setup.bash" >> ~/.bashrc
```

### 5. Verificar Instalação

```bash
# Verificar pacotes compilados
rospack list | grep mobile_rob_dev
# Deve retornar: mobile_rob_dev

# Verificar variáveis de ambiente
echo $ROS_PACKAGE_PATH
# Deve incluir: /home/ubuntu/laris_wksp/src
```

## Configuração de Hardware

### 1. Permissões de Dispositivos

```bash
# Adicionar usuário aos grupos necessários
sudo usermod -aG dialout $USER
sudo usermod -aG gpio $USER
sudo usermod -aG video $USER

# Relogar para aplicar mudanças
# logout e login novamente, ou:
sudo reboot
```

### 2. Configurar Porta Serial (Arduino/Pico)

```bash
# Criar regras udev
sudo nano /etc/udev/rules.d/99-robodc.rules
```

Adicionar:
```
# Arduino/Raspberry Pi Pico (controle de motores)
KERNEL=="ttyACM*", ATTRS{idVendor}=="2e8a", MODE="0666", GROUP="dialout", SYMLINK+="pico"
KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", MODE="0666", GROUP="dialout", SYMLINK+="arduino"

# Qualquer dispositivo serial genérico
KERNEL=="ttyACM0", MODE="0666", GROUP="dialout"
```

Aplicar regras:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. Configurar Hokuyo LiDAR (Ethernet)

```bash
# Verificar interface Ethernet
ip addr show

# LiDAR geralmente vem configurado com IP estático 192.168.0.10
# Configurar interface do Raspberry Pi na mesma rede

# Editar netplan
sudo nano /etc/netplan/50-cloud-init.yaml
```

Adicionar (se eth0 for a interface conectada ao LiDAR):
```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - 192.168.0.15/24  # IP do Raspberry Pi
      # LiDAR está em 192.168.0.10
```

Aplicar:
```bash
sudo netplan apply

# Testar conectividade
ping 192.168.0.10
```

### 4. Configurar Raspberry Pi Pico (Controle de Motores)

O Raspberry Pi Pico deve estar programado para:
- Receber comandos de velocidade via serial (formato: `vl,vr\n`)
- Enviar posição odométrica via serial (formato: `x,y,theta\n`)
- Controlar motores via PWM

Porta padrão: `/dev/ttyACM0` (115200 baud)

### 5. Configurar ESP32 (Controle de LEDs)

O ESP32 controla as matrizes de LED via Bluetooth.

- Endereço MAC: `8C:AA:B5:93:69:EE`
- Comunicação via PyBluez na API REST

## Configuração de Rede ROS

### Para Operação Standalone (Robô Sozinho)

```bash
# Adicionar ao ~/.bashrc
nano ~/.bashrc

# Adicionar ao final:
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_HOSTNAME=$(hostname)
```

### Para Operação com Computador Remoto

```bash
# No Raspberry Pi 4 (robô)
export ROS_MASTER_URI=http://robodc-1gen.local:11311
export ROS_IP=192.168.1.100  # IP do Raspberry Pi

# No computador de desenvolvimento
export ROS_MASTER_URI=http://robodc-1gen.local:11311
export ROS_IP=192.168.1.50  # IP do seu computador
```

## Instalação da API REST (Hugo-Souza/RoboDC_api)

A API REST é executada no Raspberry Pi 4 e fornece endpoints HTTP para o aplicativo móvel.

### 1. Clonar Repositório da API

```bash
cd ~
git clone https://github.com/Hugo-Souza/RoboDC_api.git
cd RoboDC_api
```

### 2. Instalar Dependências Python

```bash
# Criar ambiente virtual (opcional, mas recomendado)
python3 -m venv venv
source venv/bin/activate

# Instalar dependências
pip3 install -r requirements.txt

# OU manualmente:
pip3 install Flask==2.3.0 Flask-RESTX flask-cors
pip3 install rospkg rospy actionlib
pip3 install pybluez
```

### 3. Configurar Bluetooth (para controle de LEDs via ESP32)

```bash
# Instalar bluez
sudo apt install bluetooth bluez libbluetooth-dev -y

# Habilitar serviço Bluetooth
sudo systemctl enable bluetooth
sudo systemctl start bluetooth

# Verificar dispositivos Bluetooth
hcitool scan
# Deve detectar ESP32: 8C:AA:B5:93:69:EE
```

### 4. Testar API

```bash
# Iniciar ROS
roscore &

# Em outro terminal, iniciar API
cd ~/RoboDC_api
python3 app.py

# API deve estar rodando em http://localhost:5000

# Testar endpoint
curl http://localhost:5000/metadata/version
# Deve retornar: {"version": "1.2.3"}
```

## Testes de Verificação

### 1. Testar ROS

```bash
# Terminal 1: roscore
roscore

# Terminal 2: Publicar em tópico de teste
rostopic pub /test std_msgs/String "data: 'Hello ROS'" -r 1

# Terminal 3: Escutar tópico
rostopic echo /test
```

### 2. Testar LiDAR Hokuyo

```bash
# Iniciar driver do Hokuyo
rosrun urg_node urg_node _ip_address:=192.168.0.10

# Em outro terminal, verificar dados
rostopic echo /scan
# Deve mostrar dados do LaserScan

# Verificar frequência
rostopic hz /scan
# Deve ser ~10 Hz
```

### 3. Testar Comunicação Serial (Pico/Arduino)

```bash
# Testar manualmente via minicom ou screen
sudo apt install screen -y
screen /dev/ttyACM0 115200

# Deve ver mensagens vindos do Pico (x,y,theta)
# Digite comandos de velocidade: 0.1,0.1
```

### 4. Testar mobile_rob_dev

```bash
# Iniciar nó principal
rosrun mobile_rob_dev mobile_rob_dev_node

# Em outro terminal, enviar comando de velocidade
rostopic pub /robot/cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" -r 10

# Verificar odometria
rostopic echo /odom
# Deve mostrar dados de Odometry

# Verificar TF
rosrun tf tf_echo odom base_link
```

## Próximos Passos

Após a instalação e configuração:

1. **Criar/Carregar Mapa**: Veja [Implantação](./implantacao.md) para criar mapa do ambiente
2. **Configurar Navegação**: Ajustar parâmetros de navegação (costmaps, DWA)
3. **Instalar Aplicativo Móvel**: No tablet Android, instalar o app Ionic/Angular
4. **Configurar Inicialização Automática**: Ver seção de systemd em [Implantação](./implantacao.md)

## Troubleshooting

### Erro: "Unable to communicate with master"
```bash
# Verificar se roscore está rodando
pgrep -a roscore

# Verificar variáveis ROS
echo $ROS_MASTER_URI
echo $ROS_IP

# Reiniciar roscore
killall -9 roscore rosmaster
roscore
```

### Erro: "Permission denied /dev/ttyACM0"
```bash
# Verificar permissões
ls -l /dev/ttyACM0

# Adicionar usuário ao grupo dialout
sudo usermod -aG dialout $USER
# Relogar

# OU temporariamente
sudo chmod 666 /dev/ttyACM0
```

### Erro: LiDAR não conecta (Ethernet)
```bash
# Verificar conexão física (cabo Ethernet conectado)
# Verificar IP
ip addr show eth0

# Testar ping
ping 192.168.0.10

# Verificar firewall
sudo ufw status
# Se ativo, permitir conexões locais
sudo ufw allow from 192.168.0.0/24
```

### Raspberry Pi 4 superaquece
```bash
# Verificar temperatura
vcgencmd measure_temp
# Deve estar < 70°C

# Soluções:
# - Adicionar dissipador de calor
# - Adicionar ventoinha
# - Reduzir overclock (se aplicado)
```

### Build falha por falta de memória (RAM)
```bash
# Compilar com apenas 1 job (mais lento, mas usa menos RAM)
catkin build -j1

# OU com catkin_make
catkin_make -j1
```

## Recursos Adicionais

- [Documentação ROS Noetic](http://wiki.ros.org/noetic)
- [Repositório vivaldini/ROBO_DC](https://github.com/vivaldini/ROBO_DC)
- [Repositório Hugo-Souza/RoboDC_api](https://github.com/Hugo-Souza/RoboDC_api)
- [Hokuyo URG Node](http://wiki.ros.org/urg_node)

---

**Próximo**: [Estrutura do Repositório](./estrutura-repositorio.md) | [Implantação](./implantacao.md)

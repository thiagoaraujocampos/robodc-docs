---
sidebar_position: 1
---

# Pré-requisitos - 1ª Geração

## Sistema Operacional

**Ubuntu 20.04 LTS (Focal Fossa)** - obrigatório para ROS 1 Noetic

- Arquitetura: AMD64 (x86_64) ou ARM64 (para Raspberry Pi 4)
- Instalação: Desktop ou Server (mínimo 20GB de espaço em disco)

:::warning Compatibilidade
ROS 1 Noetic **não** é compatível com Ubuntu 22.04 ou versões superiores. Use exclusivamente Ubuntu 20.04 LTS.
:::

## ROS Noetic

### Instalação do ROS 1 Noetic

```bash
# Configurar sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Adicionar chave
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Atualizar índice de pacotes
sudo apt update

# Instalação completa (recomendado)
sudo apt install ros-noetic-desktop-full

# Configurar environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Instalar dependências
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Inicializar rosdep
sudo rosdep init
rosdep update
```

## Pacotes ROS Adicionais

Instale os pacotes necessários para o RobôDC:

```bash
# Navegação
sudo apt install ros-noetic-navigation \
                 ros-noetic-move-base \
                 ros-noetic-amcl \
                 ros-noetic-gmapping \
                 ros-noetic-map-server

# Localização e fusão sensorial
sudo apt install ros-noetic-robot-localization

# Sensores
sudo apt install ros-noetic-rplidar-ros \
                 ros-noetic-usb-cam \
                 ros-noetic-imu-filter-madgwick

# Visualização
sudo apt install ros-noetic-rviz \
                 ros-noetic-rqt \
                 ros-noetic-rqt-common-plugins

# Ferramentas de desenvolvimento
sudo apt install ros-noetic-rosserial \
                 ros-noetic-rosserial-arduino
```

## Ferramentas de Desenvolvimento

### Git
```bash
sudo apt install git
git config --global user.name "Seu Nome"
git config --global user.email "seu.email@example.com"
```

### Catkin Tools
```bash
sudo apt install python3-catkin-tools
```

### VS Code (Opcional, mas recomendado)
```bash
sudo snap install --classic code

# Extensões recomendadas:
# - ROS (Microsoft)
# - C/C++ (Microsoft)
# - Python (Microsoft)
# - CMake Tools
```

## Hardware Específico

### Para Raspberry Pi 4

```bash
# Expandir sistema de arquivos
sudo raspi-config
# Escolha: Advanced Options → Expand Filesystem

# Instalar GPIO libraries
sudo apt install python3-rpi.gpio

# Configurar permissões de portas seriais
sudo usermod -aG dialout $USER
sudo usermod -aG gpio $USER
```

### Configuração de Portas USB

```bash
# Criar regras udev para dispositivos
sudo nano /etc/udev/rules.d/99-robodc.rules
```

Adicione as seguintes regras:
```
# RPLidar
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="rplidar"

# Arduino (Base Controller)
KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", MODE:="0666", GROUP:="dialout", SYMLINK+="arduino"

# Câmera USB
KERNEL=="video*", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="082d", MODE:="0666", GROUP:="video", SYMLINK+="camera"
```

Recarregue as regras:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Rede e Conectividade

### Configuração ROS Master
Para operar com computador remoto:

```bash
# No robô (Raspberry Pi/NUC)
export ROS_MASTER_URI=http://IP_DO_ROBO:11311
export ROS_IP=IP_DO_ROBO

# No computador de desenvolvimento
export ROS_MASTER_URI=http://IP_DO_ROBO:11311
export ROS_IP=IP_DO_COMPUTADOR
```

Adicione ao `~/.bashrc` para tornar permanente.

### SSH (Para acesso remoto)
```bash
sudo apt install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
```

## Verificação da Instalação

```bash
# Verificar versão do ROS
rosversion -d
# Deve retornar: noetic

# Verificar variáveis de ambiente
printenv | grep ROS
# Deve mostrar: ROS_DISTRO=noetic, ROS_VERSION=1, etc.

# Testar roscore
roscore
# Ctrl+C para sair

# Verificar tópicos
rostopic list
```

## Conhecimentos Recomendados

- **Linux básico**: Comandos de terminal, gerenciamento de arquivos
- **ROS 1**: Conceitos de nós, tópicos, serviços, launch files
- **Python/C++**: Para modificar ou criar novos nós
- **Git**: Para controle de versão e colaboração

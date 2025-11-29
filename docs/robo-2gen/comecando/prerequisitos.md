---
sidebar_position: 1
---

# Pré-requisitos - 2ª Geração

## Versão do ROS 2

Este projeto requer **ROS 2 Humble Hawksbill** (ou especificar a versão exata utilizada).

### Sistema Operacional Recomendado
- Ubuntu 22.04 LTS (Jammy Jellyfish)

### Diferenças em Relação à 1ª Geração

| Aspecto | 1ª Geração (ROS 1) | 2ª Geração (ROS 2) |
|---------|-------------------|-------------------|
| Build System | catkin_make / catkin build | colcon |
| Launch Files | XML | Python |
| Parâmetros | rospar am | param files YAML + CLI |
| Master | roscore (necessário) | Não necessário (DDS) |

## Dependências Novas/Diferentes da 1ª Geração

### Ferramentas Essenciais

```bash
sudo apt update
sudo apt upgrade -y

# Ferramentas de desenvolvimento
sudo apt install -y \
    build-essential \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget
```

### Instalação do ROS 2 Humble

```bash
# Configurar locale
locale  # verificar se UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Adicionar repositório ROS 2
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop

# Instalar ferramentas de desenvolvimento
sudo apt install -y \
    ros-dev-tools \
    ros-humble-ros-base \
    ros-humble-ros2bag \
    ros-humble-rosbag2-storage-default-plugins
```

### Dependências ROS 2 Específicas

```bash
# Nav2 (Navigation 2)
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# SLAM Toolbox
sudo apt install -y ros-humble-slam-toolbox

# Cartographer (alternativa de SLAM)
sudo apt install -y ros-humble-cartographer ros-humble-cartographer-ros

# Robot Localization
sudo apt install -y ros-humble-robot-localization

# Controle
sudo apt install -y \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers

# Sensores
sudo apt install -y \
    ros-humble-usb-cam \
    ros-humble-realsense2-camera \
    ros-humble-urg-node

# Visualização
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins

# TF2
sudo apt install -y ros-humble-tf2-tools

# Diagnósticos
sudo apt install -y ros-humble-diagnostic-updater
```

### Dependências Python

```bash
pip3 install \
    numpy \
    opencv-python \
    transforms3d \
    pyyaml \
    scipy
```

### Dependências Novas (não presentes na 1ª geração)

#### Cyclone DDS (Recomendado para melhor performance)

```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp

# Configurar como RMW padrão
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

#### Behavior Trees (para navegação avançada)

```bash
sudo apt install -y ros-humble-behaviortree-cpp-v3
```

#### Lifecycle Nodes

```bash
sudo apt install -y ros-humble-rclcpp-lifecycle
```

## Ferramentas Necessárias

### Desenvolvimento
- **IDE**: Visual Studio Code com extensões ROS 2
  ```bash
  # Instalar VSCode extensions recomendadas:
  # - ROS
  # - Python
  # - C/C++
  # - CMake Tools
  ```

### Simulação
- **Gazebo Fortress/Garden** (compatível com ROS 2 Humble)
  ```bash
  sudo apt install -y ros-humble-gazebo-ros-pkgs
  ```

### Visualização
- **RViz2** (já incluído no ros-humble-desktop)
- **PlotJuggler** (para visualização de dados)
  ```bash
  sudo apt install -y ros-humble-plotjuggler-ros
  ```

### Controle de Versão
- Git 2.34+
  ```bash
  sudo apt install -y git git-lfs
  ```

### Build e Testes
- **Colcon** (já instalado anteriormente)
- **colcon-test**
  ```bash
  pip3 install colcon-test
  ```

## Verificação da Instalação

### 1. Verificar Instalação do ROS 2

```bash
# Source do ambiente
source /opt/ros/humble/setup.bash

# Verificar versão
ros2 --version

# Verificar variáveis de ambiente
printenv | grep ROS

# Testar comunicação
# Terminal 1:
ros2 run demo_nodes_cpp talker

# Terminal 2:
ros2 run demo_nodes_cpp listener
```

### 2. Testar Ferramentas

```bash
# Testar colcon
colcon --version

# Testar rosdep
rosdep --version

# Listar pacotes instalados
ros2 pkg list

# Verificar DDS
ros2 daemon status
```

### 3. Verificar RMW (ROS Middleware)

```bash
# Ver implementação atual
echo $RMW_IMPLEMENTATION

# Listar implementações disponíveis
ros2 doctor --report | grep middleware
```

## Configuração do Ambiente

### Configurar ~/.bashrc

```bash
# Adicionar ao ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc

# Para workspace específico (depois de compilar)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

source ~/.bashrc
```

### Configurar Cyclone DDS (Opcional mas Recomendado)

Criar arquivo de configuração `~/cyclonedds.xml`:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
        </General>
    </Domain>
</CycloneDDS>
```

Adicionar ao ~/.bashrc:
```bash
echo "export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml" >> ~/.bashrc
```

## Problemas Comuns

### Problema 1: ROS 2 não encontrado

**Solução**:
```bash
source /opt/ros/humble/setup.bash
```

### Problema 2: Erro de comunicação DDS

**Diagnóstico**:
```bash
ros2 doctor --report
ros2 daemon stop
ros2 daemon start
```

**Solução**:
- Verificar firewall
- Configurar ROS_DOMAIN_ID único
- Verificar configuração de rede

### Problema 3: colcon build falha

**Solução**:
```bash
# Limpar build
rm -rf build install log

# Instalar dependências
rosdep install --from-paths src --ignore-src -r -y

# Compilar com verbose
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+
```

### Problema 4: Import error em Python

**Solução**:
```bash
# Verificar se sourced corretamente
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Verificar PYTHONPATH
echo $PYTHONPATH
```

## Hardware Requirements (para execução no robô real)

### Mínimo Recomendado
- **CPU**: 4 cores, 2.0 GHz+
- **RAM**: 4 GB (8 GB recomendado)
- **Armazenamento**: 32 GB SSD
- **Rede**: Ethernet Gigabit ou WiFi 5 GHz

### Recomendado para Melhor Performance
- **CPU**: 8 cores, 2.5 GHz+
- **RAM**: 8 GB ou mais
- **Armazenamento**: 64 GB+ NVMe SSD
- **GPU**: Nvidia Jetson ou similar (para processamento de visão)

## Próximos Passos

Após a instalação dos pré-requisitos:
1. Clonar o repositório da 2ª geração
2. Instalar dependências específicas do projeto
3. Compilar o workspace
4. Executar testes básicos

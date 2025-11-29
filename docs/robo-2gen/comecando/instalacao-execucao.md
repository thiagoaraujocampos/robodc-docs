---
sidebar_position: 2
---

# Como Clonar, Instalar e Executar - 2ª Geração

## Clonar o Repositório

### 1. Criar Workspace do ROS 2

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clonar o Repositório

```bash
git clone [URL_DO_REPOSITORIO] robodc-2gen
cd robodc-2gen

# Checkout da branch/tag desejada
git checkout [BRANCH_OU_TAG]
```

### 3. Clonar Dependências (se houver submódulos)

```bash
git submodule update --init --recursive

# Ou usar vcs tool (se houver arquivo .repos)
vcs import < robodc.repos
```

## Instalar Dependências

### 1. Source do ROS 2

```bash
source /opt/ros/humble/setup.bash
```

### 2. Atualizar rosdep

```bash
cd ~/ros2_ws
sudo rosdep init  # apenas na primeira vez
rosdep update
```

### 3. Instalar Dependências do Projeto

```bash
# Instalar todas as dependências automaticamente
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Instalar Dependências Específicas (se houver)

```bash
cd ~/ros2_ws/src/robodc-2gen

# Se houver script de instalação
./scripts/install_dependencies.sh

# Instalar dependências Python específicas
pip3 install -r requirements.txt
```

## Compilar o Projeto

### 1. Compilar com Colcon

```bash
cd ~/ros2_ws

# Compilação básica
colcon build

# Compilação recomendada (com symlinks para desenvolvimento)
colcon build --symlink-install

# Compilação com Release (melhor performance)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Compilação verbose (para debug)
colcon build --event-handlers console_direct+
```

### 2. Compilar Pacotes Específicos

```bash
# Compilar apenas um pacote
colcon build --packages-select robodc_navigation

# Compilar pacote e suas dependências
colcon build --packages-up-to robodc_navigation
```

### 3. Configurar Ambiente

```bash
source ~/ros2_ws/install/setup.bash
```

**Dica**: Adicione ao `~/.bashrc`:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Verificar Compilação

```bash
# Listar pacotes instalados
ros2 pkg list | grep robodc

# Ver executáveis disponíveis
ros2 pkg executables robodc_navigation
```

## Executar o Projeto

### Modo Simulação

#### Iniciar Gazebo com o Robô

```bash
# Terminal 1: Lançar simulação
ros2 launch robodc_2gen simulation.launch.py

# Aguardar Gazebo carregar completamente
```

#### Lançar Navegação

```bash
# Terminal 2: Lançar stack de navegação
ros2 launch robodc_navigation navigation.launch.py use_sim_time:=true
```

#### Visualizar em RViz2

```bash
# Terminal 3: Abrir RViz2
ros2 launch robodc_2gen rviz.launch.py
```

#### Enviar Goal

```bash
# Via linha de comando
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"

# Ou use RViz2: "2D Goal Pose" tool
```

### Modo Robô Real

#### Checklist Pré-operação
- [ ] Robô ligado e conectado à rede
- [ ] Bateria carregada (>30%)
- [ ] Área de operação livre de obstáculos
- [ ] Botão de emergência acessível

#### Iniciar Sistema do Robô

```bash
# Lançar todos os sistemas (sensores + controle + navegação)
ros2 launch robodc_2gen robot.launch.py
```

#### Verificar Status

```bash
# Em outro terminal, verificar nós ativos
ros2 node list

# Verificar tópicos
ros2 topic list

# Ver frequência de publicação
ros2 topic hz /scan
ros2 topic hz /odom

# Verificar transforms
ros2 run tf2_tools view_frames
```

#### Modo Manual (Teleoperação)

```bash
# Controle por teclado
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel

# Ou joystick (se configurado)
ros2 launch robodc_2gen teleop_joystick.launch.py
```

## Testes Rápidos

### 1. Testar Comunicação de Tópicos

```bash
# Ver mensagens de um tópico
ros2 topic echo /scan

# Ver info de um tópico
ros2 topic info /scan -v

# Publicar comando de velocidade de teste
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10
```

### 2. Testar Serviços

```bash
# Listar serviços disponíveis
ros2 service list

# Chamar serviço de teste
ros2 service call /calibrate_sensors std_srvs/srv/Trigger
```

### 3. Testar Transformações

```bash
# Ver transformação entre frames
ros2 run tf2_ros tf2_echo base_link camera_link

# Visualizar árvore de transforms
ros2 run tf2_tools view_frames
evince frames.pdf
```

### 4. Executar Testes Automatizados

```bash
# Executar todos os testes
colcon test

# Ver resultados
colcon test-result --all

# Executar testes de um pacote específico
colcon test --packages-select robodc_navigation

# Executar com verbose
colcon test --event-handlers console_direct+
```

## Estrutura de Launch Files

### Launch Files Principais

#### `robot.launch.py`
```python
# Lança todos os sistemas do robô real
ros2 launch robodc_2gen robot.launch.py
```

Argumentos disponíveis:
- `use_sim_time`: false (padrão para robô real)
- `params_file`: Arquivo de parâmetros customizado
- `namespace`: Namespace para os nós

#### `simulation.launch.py`
```python
# Lança simulação Gazebo
ros2 launch robodc_2gen simulation.launch.py
```

Argumentos:
- `world`: Arquivo de mundo Gazebo
- `robot_name`: Nome do robô na simulação
- `spawn_x`, `spawn_y`: Posição inicial

#### `navigation.launch.py`
```python
# Lança apenas o stack de navegação
ros2 launch robodc_navigation navigation.launch.py use_sim_time:=false
```

### Lançar com Argumentos Personalizados

```bash
# Exemplo: lançar com namespace e parâmetros custom
ros2 launch robodc_2gen robot.launch.py \
  namespace:=/robot1 \
  params_file:=/path/to/custom_params.yaml
```

## Logs e Diagnóstico

### Visualizar Logs

```bash
# Logs em tempo real de um nó específico
ros2 run rqt_console rqt_console

# Ver logs salvos
cd ~/.ros/log/
ls -ltr
```

### Usar ros2 doctor

```bash
# Diagnóstico geral do sistema
ros2 doctor

# Relatório completo
ros2 doctor --report
```

### Monitoramento em Tempo Real

```bash
# Ver gráfico de nós e tópicos
ros2 run rqt_graph rqt_graph

# Monitor de recursos
ros2 run rqt_top rqt_top
```

## Solução de Problemas

### Erro: "Package not found"

```bash
# Verificar se o workspace foi sourced
source ~/ros2_ws/install/setup.bash

# Recompilar o pacote
cd ~/ros2_ws
colcon build --packages-select [NOME_DO_PACOTE]
source install/setup.bash
```

### Erro: "No transform available"

```bash
# Verificar se robot_state_publisher está rodando
ros2 node list | grep robot_state_publisher

# Verificar URDF
ros2 param get /robot_state_publisher robot_description

# Ver árvore de TF
ros2 run tf2_tools view_frames
```

### Erro: "DDS communication failure"

```bash
# Verificar RMW
echo $RMW_IMPLEMENTATION

# Verificar domínio
echo $ROS_DOMAIN_ID

# Reiniciar daemon
ros2 daemon stop
ros2 daemon start

# Testar comunicação
ros2 topic list
```

### Performance Baixa

```bash
# Verificar QoS dos tópicos
ros2 topic info /scan -v

# Mudar para Cyclone DDS (se não estiver usando)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Recompilar com Release
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Sensores não Publicam

```bash
# Verificar se dispositivos estão conectados
ls -l /dev/video* /dev/ttyUSB* /dev/ttyACM*

# Verificar permissões
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
# Logout e login necessário

# Ver logs do nó do sensor
ros2 run rqt_console rqt_console
# Filtrar por nó específico
```

## Desenvolvimento

### Compilação Incremental

```bash
# Recompilar apenas pacotes modificados
colcon build --packages-select robodc_navigation

# Com symlink (não precisa recompilar Python)
colcon build --symlink-install
```

### Debug

```bash
# Lançar nó com debugging
ros2 run --prefix 'gdb -ex run --args' robodc_navigation navigation_node

# Ou com xterm
ros2 run --prefix 'xterm -e gdb -ex run --args' robodc_navigation navigation_node
```

### Profiling

```bash
# Usar valgrind
ros2 run --prefix 'valgrind --tool=callgrind' robodc_navigation navigation_node

# Usar perf
ros2 run --prefix 'perf record -g' robodc_navigation navigation_node
```

## Próximos Passos

Após conseguir executar o projeto:
1. Explorar a estrutura do repositório
2. Entender os nós principais
3. Revisar parâmetros de configuração
4. Executar cenários de teste
5. Contribuir com melhorias

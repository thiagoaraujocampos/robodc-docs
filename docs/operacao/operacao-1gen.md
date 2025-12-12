---
sidebar_position: 1
---

# Como Operar o Robô de 1ª Geração

## Preparação Pré-Operação

### Checklist Antes de Ligar

- [ ] Verificar carga da bateria (mínimo 30%)
- [ ] Inspecionar visualmente o robô (danos, conexões soltas)
- [ ] Verificar área de operação livre de obstáculos
- [ ] Garantir que botão de emergência está acessível
- [ ] Confirmar que sensores estão limpos

### Ligar o Sistema

1. **Conectar a bateria**
2. **Ligar o computador de bordo**
3. **Aguardar inicialização** (aproximadamente 30-60 segundos)
4. **Verificar LEDs de status** (se aplicável)

## Iniciar o Software

### Método 1: Inicialização Automática

Se o serviço systemd está configurado, o robô inicia automaticamente.

Verificar status:
```bash
systemctl status robodc
```

### Método 2: Inicialização Manual

```bash
# SSH no robô ou terminal local
ssh usuario@[IP_DO_ROBO]

# Source do ambiente
source ~/catkin_ws/devel/setup.bash

# Lançar o robô
roslaunch robodc_1gen robot.launch
```

## Modos de Operação

### Modo Manual (Teleoperação)

```bash
# Controle por teclado
rosrun teleop_twist_keyboard teleop_twist_keyboard
```

**Teclas de controle**:
- `i`: Frente
- `,`: Trás
- `j`: Girar esquerda
- `l`: Girar direita
- `k`: Parar
- `q/z`: Aumentar/diminuir velocidade linear
- `w/x`: Aumentar/diminuir velocidade angular

### Modo Autônomo

```bash
# Iniciar navegação autônoma
roslaunch robodc_navigation autonomous.launch

# Enviar objetivo via RViz:
# 1. Abrir RViz
# 2. Usar ferramenta "2D Nav Goal"
# 3. Clicar e arrastar no mapa
```

### Modo Patrol (se implementado)

```bash
roslaunch robodc_navigation patrol.launch waypoints_file:=patrol_points.yaml
```

## Monitoramento Durante Operação

### Verificar Status

```bash
# Ver todos os nós ativos
rosnode list

# Verificar tópicos
rostopic list

# Monitorar bateria
rostopic echo /battery_state

# Ver diagnósticos
rostopic echo /diagnostics
```

### Usar RViz para Visualização

```bash
rosrun rviz rviz -d $(rospack find robodc_bringup)/rviz/robot.rviz
```

**Elementos visualizados**:
- Modelo do robô
- Scan do LIDAR
- Mapa (se usando navegação)
- Trajetória planejada
- Transformações (TF)

## Procedimentos Operacionais

### Iniciar Missão de Navegação

1. **Verificar localização do robô no mapa**
2. **Definir objetivo** (via RViz ou API)
3. **Aguardar confirmação de planejamento**
4. **Monitorar progresso**
5. **Confirmar chegada ao objetivo**

### Criar/Atualizar Mapa (SLAM)

```bash
# 1. Iniciar SLAM
roslaunch robodc_navigation slam.launch

# 2. Teleoperar o robô pela área
rosrun teleop_twist_keyboard teleop_twist_keyboard

# 3. Visualizar mapa sendo criado em RViz

# 4. Salvar mapa quando satisfatório
rosrun map_server map_saver -f ~/maps/meu_mapa
```

### Calibrar Sensores

```bash
# Calibração da câmera
rosrun camera_calibration cameracalibrator.py \
  --size 8x6 --square 0.108 \
  image:=/camera/image_raw camera:=/camera

# Calibração IMU (seguir procedimento específico)
rosservice call /calibrate_imu
```

## Parada e Desligamento

### Parada de Emergência

**Física**: Pressionar botão de emergência vermelho

**Software**:
```bash
rosservice call /emergency_stop
```

### Desligamento Normal

1. **Parar navegação** (Ctrl+C no terminal do launch ou)
   ```bash
   rosnode kill /navigation_node
   ```

2. **Parar todos os nós**
   ```bash
   # Se usando launch file, Ctrl+C
   # Ou matar roscore
   killall -9 roscore rosmaster
   ```

3. **Desligar computador**
   ```bash
   sudo shutdown -h now
   ```

4. **Desconectar bateria** (após desligamento completo)

## Troubleshooting Operacional

### Robô Não Responde a Comandos

**Verificar**:
```bash
# Comunicação ROS funcionando?
rostopic list

# Nó de controle ativo?
rosnode list | grep motor_controller

# Publicar comando de teste
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" --once
```

### Localização Perdida

**Solução**:
```bash
# Resetar localização
rosservice call /reset_localization

# Ou definir pose inicial manualmente no RViz
# Usar ferramenta "2D Pose Estimate"
```

### Sensor Não Está Funcionando

**Verificar**:
```bash
# Tópico está publicando?
rostopic hz /scan

# Ver mensagens de erro
rostopic echo /rosout | grep ERROR
```

### Bateria Baixa Durante Operação

**Procedimento**:
1. Robô deve alertar automaticamente em 20%
2. Finalizar missão atual
3. Retornar à base (se implementado) ou
4. Parar em local seguro
5. Desligar sistema

## Boas Práticas

### Durante Operação
- ✅ Sempre manter botão de emergência acessível
- ✅ Supervisionar operação autônoma
- ✅ Verificar status da bateria regularmente
- ✅ Manter área de operação adequada
- ❌ Não operar com bateria abaixo de 20%
- ❌ Não deixar robô operando sozinho por períodos longos
- ❌ Não operar em condições ambientais inadequadas

### Manutenção Regular
- Limpar sensores após cada uso
- Verificar conexões periodicamente
- Carregar bateria completamente após uso
- Registrar ocorrências em log

## Logs de Operação

### Acessar Logs

```bash
# Logs ROS
cd ~/.ros/log/latest/
ls -lh

# Ver log de um nó específico
cat ~/.ros/log/latest/navigation_node*.log
```

### Registrar Operação

Manter registro de:
- Data/hora de operação
- Duração
- Tarefas realizadas
- Problemas encontrados
- Nível de bateria início/fim

## Vídeos e Tutoriais

Vídeos demonstrativos e tutoriais específicos serão disponibilizados conforme forem produzidos pela equipe:

- Tutorial: Primeira Inicialização
- Tutorial: Navegação Autônoma
- Tutorial: Criação de Mapas
- Tutorial: Solução de Problemas Comuns

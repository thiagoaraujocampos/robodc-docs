---
sidebar_position: 2
---

# Hardware

## Plataforma Robótica

### Descrição Geral
O RobôDC de primeira geração utiliza uma plataforma robótica diferencial customizada, construída em estrutura MDF de dois níveis, projetada especificamente para navegação em ambientes internos do Departamento de Computação da UFSCar.

### Especificações da Base
- **Estrutura**: Base MDF 10mm em dois níveis
- **Sistema de Locomoção**: Tração diferencial (2 rodas motrizes)
- **Velocidade Máxima**: 0.5 m/s (linear), 1.0 rad/s (angular)
- **Autonomia**: Aproximadamente 2-3 horas de operação contínua
- **Ambiente de Operação**: Ambientes internos do DC-UFSCar

### Arquitetura Computacional

O sistema computacional do RobôDC de 1ª geração é distribuído em três unidades de processamento:

#### 1. Raspberry Pi 4 Model B (Computador Principal)
- **Função**: Processamento central, execução do ROS 1 Noetic, API REST, gateway NAT
- **Processador**: Broadcom BCM2711, Quad-core Cortex-A72 (ARM v8) 64-bit @ 1.5GHz
- **Memória RAM**: 4GB LPDDR4-3200 SDRAM
- **Armazenamento**: Cartão microSD (mínimo 32GB, recomendado 64GB)
- **Sistema Operacional**: Ubuntu Server 20.04 LTS (64-bit)
- **Conectividade**: 
  - Ethernet Gigabit
  - Wi-Fi 802.11ac (2.4/5GHz)
  - Bluetooth 5.0
  - 2x USB 3.0, 2x USB 2.0
- **Responsabilidades**:
  - Execução do stack ROS 1 (nodes, topics, services, actions)
  - Processamento de dados do LiDAR Hokuyo
  - API REST Flask para comunicação com aplicativo móvel
  - Gateway NAT para acesso à internet
  - Gerenciamento da rede MrRoboto

#### 2. Raspberry Pi Pico (Controlador de Motores)
- **Função**: Controle de baixo nível dos motores
- **Processador**: RP2040 dual-core ARM Cortex-M0+ @ 133MHz
- **Memória**: 264KB SRAM, 2MB Flash
- **Interface**: GPIO, PWM, I2C, SPI, UART
- **Responsabilidades**:
  - Geração de sinais PWM para controle de velocidade dos motores
  - Interface com drivers de motor
  - Leitura de encoders (se aplicável)
  - Comunicação serial com Raspberry Pi 4 (via USB ou UART)

#### 3. ESP32 (Controlador de Interface Visual)
- **Função**: Controle das matrizes de LED (face digital do robô)
- **Processador**: Xtensa dual-core 32-bit LX6 @ 240MHz
- **Memória**: 520KB SRAM
- **Conectividade**: Wi-Fi 802.11 b/g/n, Bluetooth Classic e BLE
- **Responsabilidades**:
  - Controle das matrizes de LED para expressões faciais
  - Comunicação via Bluetooth com o aplicativo móvel
  - Recepção de comandos de expressões faciais
  - Execução de animações e padrões visuais

## Sensores

### LiDAR

#### Hokuyo URG-04LX-UG01 (ou modelo similar da série URG)
- **Fabricante**: Hokuyo Automatic Co., Ltd.
- **Alcance**: 20mm a 5600mm (5.6 metros)
- **Precisão**: ±10mm (para distâncias de 60-1000mm), ±1% da distância medida (1000-4095mm)
- **Resolução Angular**: 0.36° (1024 passos em 240°)
- **Ângulo de Varredura**: 240° (±120° do centro)
- **Taxa de Varredura**: 10 Hz (100ms/scan)
- **Interface**: Ethernet (RJ45) - conexão direta com Raspberry Pi 4
- **Alimentação**: 12V DC fornecido por bateria dedicada
- **Posição no Robô**: Nível superior da estrutura, centralmente posicionado
- **Uso**: Navegação autônoma, mapeamento SLAM, detecção e desvio de obstáculos
- **Características**:
  - Sensor laser 2D de alta precisão
  - Ideal para ambientes internos
  - Não detecta superfícies transparentes ou espelhadas
  - Possui bateria VRLA 12V 7Ah dedicada

### Câmera (via Tablet Android)

- **Dispositivo**: Câmera do tablet Android Samsung
- **Função**: Detecção de faces e expressões faciais
- **Tecnologia**: Face-API.js (TensorFlow.js)
- **Processamento**: No próprio tablet (edge computing)
- **Interface**: Comunicação via API REST com Raspberry Pi 4
- **Aplicação**: 
  - Identificação de usuários
  - Detecção de expressões emocionais
  - Interação social aprimorada
- **Características**:
  - Resolução HD (depende do modelo do tablet)
  - Processamento em tempo real no dispositivo
  - Baixa latência para interação

## Atuadores

### Motores

#### Motores DC de Tração
- **Quantidade**: 2 motores (tração diferencial)
- **Tipo**: Motores DC com caixa de redução
- **Tensão de Operação**: 12V DC
- **Corrente**: Variável conforme carga
- **Controle**: PWM via Raspberry Pi Pico
- **Configuração**: Um motor para cada roda traseira
- **Responsabilidades**:
  - Locomoção do robô (frente, trás, rotação)
  - Controle independente para manobras diferenciais
  - Resposta a comandos `/cmd_vel` via ROS

### Drivers de Motor

- **Controlador**: Raspberry Pi Pico
- **Interface**: Sinais PWM gerados pelo RP2040
- **Controle de Velocidade**: Modulação por largura de pulso (PWM)
- **Direção**: Controle via GPIO (pinos digitais)
- **Proteções**: Implementadas no hardware do driver
- **Comunicação**: Serial (UART/USB) com Raspberry Pi 4

### Matrizes de LED (Face Digital)

#### Sistema de Expressões Faciais
- **Controlador**: ESP32
- **Tipo**: Matrizes de LEDs organizadas para formar face digital
- **Função**: Exibir expressões faciais para interação social
- **Conectividade**: Bluetooth (comunicação com aplicativo móvel)
- **Expressões Disponíveis**: 
  - 45 expressões diferentes (códigos 0-44)
  - Incluem: feliz, triste, neutro, surpreso, pensativo, etc.
- **Características**:
  - Controle individual de cada LED
  - Animações e transições suaves
  - Resposta a comandos via API REST (`/led/changeExpression/{expressionNumber}`)
  - Baixo consumo de energia

## Sistema de Energia

### Configuração de Baterias

O RobôDC de 1ª geração utiliza **duas baterias VRLA independentes** para garantir operação estável e segura:

#### Bateria 1: LiDAR Hokuyo (Dedicada)
- **Tipo**: VRLA (Valve Regulated Lead Acid) - Bateria selada
- **Tensão Nominal**: 12V DC
- **Capacidade**: 7Ah
- **Função**: Alimentação exclusiva do LiDAR Hokuyo
- **Justificativa**: LiDAR requer alimentação estável e isolada para evitar interferências
- **Características**:
  - Manutenção zero (selada)
  - Operação em qualquer posição
  - Baixa autodescarga
  - Interruptor dedicado para ligar/desligar

#### Bateria 2: Sistema Geral
- **Tipo**: VRLA (Valve Regulated Lead Acid) - Bateria selada
- **Tensão Nominal**: 12V DC
- **Capacidade**: 7Ah
- **Função**: Alimentação dos demais componentes
- **Alimenta**:
  - Raspberry Pi 4 (via conversor DC/DC 12V→5V)
  - Raspberry Pi Pico (via USB da RPi4 ou conversor separado)
  - ESP32 (via conversor DC/DC 12V→5V ou 3.3V)
  - Motores DC (via conversor ou direto a 12V)
  - Outros periféricos
- **Interruptor**: Independente da bateria do LiDAR

### Sistema de Gerenciamento de Energia

#### Conversores DC/DC Step Down
- **Função**: Reduzir 12V da bateria para tensões operacionais dos componentes
- **Conversores necessários**:
  - **12V → 5V/3A**: Para Raspberry Pi 4 (via USB-C ou GPIO)
  - **12V → 5V/1A**: Para ESP32 (ou 3.3V direto)
  - **12V → 5V**: Para periféricos USB (se necessário)
- **Tipo**: Buck converters (step-down)
- **Eficiência**: ~90% (típica para conversores de qualidade)
- **Proteções Integradas**:
  - Sobrecorrente
  - Sobretensão
  - Curto-circuito
  - Sobretemeperatura

#### Proteções e Segurança
- **Interruptores Individuais**: 
  - Interruptor 1: Bateria do LiDAR
  - Interruptor 2: Bateria do sistema geral
  - Permite ligar/desligar componentes independentemente
- **Botão de Emergência**: 
  - Corta alimentação imediatamente em caso de emergência
  - Tipo: Botão vermelho de pressão/giro para liberar
  - Conexão: Em série com as baterias (desliga ambas)
- **Fusíveis**: Proteção contra sobrecorrente
- **Monitoramento** (opcional): 
  - Sensor de tensão (divisor resistivo + ADC do RPi4)
  - Alerta de bateria baixa via software
  - Desligamento seguro quando tensão crítica

### Autonomia e Recarga

#### Autonomia Estimada
- **Condições Normais**: 2-3 horas de operação contínua
- **Fatores que afetam**:
  - Uso intensivo dos motores (navegação constante)
  - Processamento pesado (SLAM, visão computacional)
  - Uso das matrizes de LED (brilho alto)
  - Estado de carga das baterias

#### Recarga
- **Tipo**: Carregadores específicos para baterias VRLA 12V
- **Tempo de Recarga**: 4-8 horas (dependendo do descarregamento)
- **Método**: Remoção das baterias ou carregamento in-loco (se houver conector externo)
- **Indicação**: LED de status no carregador (vermelho=carregando, verde=completo)

## Conectividade e Comunicação

### Rede Wi-Fi Dedicada

#### Roteador MrRoboto
- **Função**: Rede Wi-Fi privada e dedicada para o robô
- **SSID**: MrRoboto
- **Finalidade**:
  - Comunicação entre Raspberry Pi 4 e aplicativo móvel (tablet/smartphone)
  - Isolamento da rede principal do DC
  - Controle total sobre configuração de rede
- **Componentes conectados**:
  - Raspberry Pi 4 (via Wi-Fi ou Ethernet)
  - Tablet Android (aplicativo RobôDC)
  - Dispositivos de desenvolvimento/debug

### Gateway NAT (Raspberry Pi 4)

- **Função**: Fornecer acesso à internet para o robô
- **Configuração**: Raspberry Pi 4 atua como gateway NAT
- **Interfaces**:
  - **Interface Externa**: Conexão à rede do DC (Ethernet ou Wi-Fi)
  - **Interface Interna**: Rede MrRoboto (Wi-Fi ou Ethernet)
- **Utilidade**:
  - Atualizar pacotes ROS
  - Instalar dependências via `apt` ou `pip`
  - Acesso remoto SSH (se configurado)
  - Sincronização de hora (NTP)

### Interfaces de Comunicação

#### Ethernet
- **Raspberry Pi 4**: Gigabit Ethernet (RJ45)
- **LiDAR Hokuyo**: Ethernet (conexão direta com RPi4)
- **Roteador**: Porta WAN e LAN

#### Wi-Fi
- **Raspberry Pi 4**: Wi-Fi 802.11ac dual-band (2.4/5GHz)
- **ESP32**: Wi-Fi 802.11 b/g/n (2.4GHz)
- **Tablet Android**: Wi-Fi para conexão com rede MrRoboto

#### Bluetooth
- **ESP32**: Bluetooth Classic + BLE
  - Endereço MAC: `8C:AA:B5:93:69:EE`
  - Comunicação com aplicativo para controle de LEDs
  - Biblioteca PyBluez na API para envio de comandos

#### USB
- **Raspberry Pi 4**:
  - 2x USB 3.0 (alta velocidade)
  - 2x USB 2.0
  - USB-C para alimentação (5V/3A)
- **Usos**:
  - Conexão com Raspberry Pi Pico (controle de motores)
  - Periféricos (teclado/mouse para configuração)
  - Armazenamento externo (pendrive)

#### Serial (UART)
- **Comunicação RPi4 ↔ RPi Pico**: Serial via USB ou GPIO (TX/RX)
- **Protocolo**: Comandos de velocidade, leitura de encoders
- **Baud Rate**: Configurável (típico: 115200 bps)

## Estrutura Física

### Base em MDF

#### Especificações da Estrutura
- **Material**: MDF (Medium-Density Fiberboard) de 10mm de espessura
- **Configuração**: Estrutura de dois níveis
- **Vantagens**:
  - Baixo custo
  - Fácil usinagem e customização
  - Leveza (importante para autonomia)
  - Rigidez adequada para componentes eletrônicos

#### Nível Inferior (Base)
- **Componentes montados**:
  - Motores DC com rodas
  - Bateria 2 (sistema geral)
  - Conversores DC/DC
  - Raspberry Pi Pico (controlador de motores)
  - Drivers de motor
  - Interruptores e botão de emergência
- **Finalidade**: 
  - Sistema de locomoção
  - Sistema de energia
  - Controle de baixo nível

#### Nível Superior (Plataforma de Sensores)
- **Componentes montados**:
  - LiDAR Hokuyo (posição central)
  - Bateria 1 (dedicada ao LiDAR)
  - Raspberry Pi 4 (computador principal)
  - ESP32 (controle de LEDs)
  - Matrizes de LED (face digital)
  - Roteador Wi-Fi MrRoboto
  - Tablet Android (suporte/dock)
- **Finalidade**:
  - Sensoriamento
  - Processamento
  - Interface com usuário
  - Comunicação

### Dimensões e Características Físicas

- **Altura Total**: Aproximadamente 40-50cm (estimado, base + níveis + sensores)
- **Peso Total**: Aproximadamente 8-10 kg (com baterias e componentes)
- **Centro de Gravidade**: Baixo, devido às baterias no nível inferior
- **Estabilidade**: Adequada para ambientes internos planos

## Interface com Usuário

### Tablet Android

#### Especificações
- **Modelo**: Samsung Galaxy Tab (modelo não especificado no PDF)
- **Sistema Operacional**: Android
- **Aplicativo**: RobôDC (Ionic 6 + Angular 15)
- **Função**: Interface gráfica principal para interação com o robô
- **Montagem**: Fixado na estrutura superior do robô (suporte/dock)

#### Funcionalidades do Aplicativo
1. **Navegação**:
   - Seleção de destino entre 17 locais do DC
   - Visualização de status em tempo real
   - Mapa interativo
2. **Cardápio do RU**:
   - Consulta de cardápio do Restaurante Universitário
   - Integração com API externa
3. **Expressões Faciais**:
   - Seleção de 45 expressões diferentes
   - Envio via Bluetooth para ESP32
4. **Controle Manual**:
   - Joystick virtual para controle direto
   - Comandos de velocidade linear e angular

#### Conectividade
- **Wi-Fi**: Conectado à rede MrRoboto
- **Bluetooth**: Comunicação direta com ESP32 para LEDs
- **API REST**: HTTP requests para Raspberry Pi 4 (porta 5000)

## Diagrama de Conexões do Sistema

```
┌─────────────────────────────────────────────────────────────┐
│                   ROBODC - 1ª GERAÇÃO                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  NÍVEL SUPERIOR                                             │
│  ┌──────────────┐      ┌─────────────────┐                 │
│  │ LiDAR Hokuyo │◄─────┤ Bateria 1 (12V) │                 │
│  │  (Ethernet)  │      └─────────────────┘                 │
│  └──────┬───────┘                                           │
│         │ Ethernet                                          │
│  ┌──────▼─────────────┐    ┌──────────────┐                │
│  │  Raspberry Pi 4    │◄───┤ Conversor DC │◄─┐             │
│  │ (ROS 1 + API REST) │    │  12V → 5V    │  │             │
│  └─┬─────────────────┬┘    └──────────────┘  │             │
│    │ Wi-Fi/BT    USB │                        │             │
│    │             │   └──────────┐             │             │
│  ┌─▼──────┐  ┌──▼─────────┐    │             │             │
│  │ Tablet │  │ ESP32      │    │             │             │
│  │Android │  │(LED+BT)    │    │             │             │
│  │(App)   │  └────────────┘    │             │             │
│  └────────┘        ▲            │             │             │
│                    │ Bluetooth  │             │             │
│                    └────────────┘             │             │
│                                               │             │
├───────────────────────────────────────────────┼─────────────┤
│                                               │             │
│  NÍVEL INFERIOR                               │             │
│                          ┌────────────────────┘             │
│  ┌─────────────────┐     │                                  │
│  │ Bateria 2 (12V) ├─────┤                                  │
│  └─────────────────┘     │                                  │
│         │                │                                  │
│         │   ┌────────────▼───────┐                          │
│         └───►Conversores DC/DC   │                          │
│             │  12V → 5V/12V     │                          │
│             └─────┬──────────────┘                          │
│                   │                                         │
│            ┌──────▼──────────┐                              │
│            │ Raspberry Pi    │                              │
│            │ Pico (PWM)      │                              │
│            └──────┬──────────┘                              │
│                   │ PWM + GPIO                              │
│            ┌──────▼──────────┐                              │
│            │ Drivers Motor   │                              │
│            └──────┬──────────┘                              │
│                   │                                         │
│       ┌───────────┴──────────┐                              │
│       │                      │                              │
│  ┌────▼─────┐         ┌──────▼────┐                        │
│  │ Motor DC │         │ Motor DC  │                         │
│  │ Esquerdo │         │  Direito  │                         │
│  └──────────┘         └───────────┘                         │
│                                                             │
└─────────────────────────────────────────────────────────────┘

COMUNICAÇÃO:
- ROS 1 Topics/Services/Actions: Raspberry Pi 4 (master)
- HTTP API REST: Tablet ↔ RPi4 (porta 5000)
- Bluetooth: Tablet ↔ ESP32 (expressões LED)
- Serial/USB: RPi4 ↔ Pico (controle motores)
- Ethernet: LiDAR ↔ RPi4
```

## Limitações de Hardware

### Processamento
- **CPU ARM Cortex-A72**: Raspberry Pi 4 pode ter dificuldades com processamento pesado simultâneo (SLAM + visão + navegação)
- **Memória**: 4GB de RAM pode ser limitante para múltiplas tarefas ROS simultâneas
- **Armazenamento**: MicroSD pode ter taxa de I/O limitada (considerar SSD via USB 3.0 para melhor performance)

### Sensoriamento
- **LiDAR 2D**: 
  - Não detecta obstáculos fora do plano horizontal de varredura (ex: mesas, cadeiras baixas, objetos elevados)
  - Alcance limitado a 5.6 metros
  - Ângulo de varredura de 240° (não detecta objetos nas laterais e traseira)
- **Câmera via Tablet**: 
  - Dependente do posicionamento e qualidade da câmera do tablet
  - Processamento de Face-API no tablet (limitação de hardware móvel)
  - Não há câmera integrada ao robô para navegação visual
- **Ausência de IMU dedicada**: 
  - Deriva na estimativa de orientação (baseada apenas em odometria)
  - Sem correção de acelerações e velocidades angulares

### Locomoção
- **Tração diferencial**: Dificuldade em superfícies irregulares ou com atrito desigual
- **Ausência de suspensão**: Limitado a pisos planos e lisos
- **Velocidade limitada**: 0.5 m/s para garantir estabilidade e segurança
- **Manobras**: Raio de giro mínimo limitado pela base das rodas

### Energia
- **Autonomia**: 2-3 horas pode ser insuficiente para operações prolongadas ou jornadas completas
- **Tempo de recarga**: 4-8 horas (bateria VRLA tem recarga lenta)
- **Peso das baterias**: VRLA são pesadas, impactando a eficiência energética
- **Degradação**: Baterias VRLA têm vida útil de ~300-500 ciclos

### Comunicação
- **Dependência de Wi-Fi**: Operação remota depende de rede MrRoboto estável
- **Alcance limitado**: Wi-Fi pode ter interferências ou perda de sinal em ambientes com múltiplas paredes
- **Bluetooth para LEDs**: Alcance limitado (~10 metros)
- **Latência**: HTTP requests podem ter latência variável dependendo da carga de rede

### Estrutura
- **Material MDF**: 
  - Sensível à umidade (pode empenar)
  - Resistência mecânica limitada (comparado a metal/plástico)
  - Aparência menos profissional
- **Dois níveis**: Pode aumentar centro de gravidade (afeta estabilidade)

## Manutenção e Cuidados

### Manutenção Regular

#### Inspeção Visual (Semanal)
- Verificar integridade da estrutura MDF (rachaduras, empenamento)
- Inspecionar conexões de cabos (Ethernet do LiDAR, USB do Pico, alimentação)
- Checar fixação de componentes (baterias, RPi4, ESP32, LiDAR)
- Verificar estado das rodas (desgaste, alinhamento)

#### Limpeza (Quinzenal)
- **LiDAR Hokuyo**: 
  - Limpar janela de vidro com pano de microfibra
  - Remover poeira do corpo do sensor
  - Verificar que a rotação está livre de obstruções
- **Matrizes de LED**: Limpar superfície com pano seco
- **Ventilação**: Limpar aberturas de ventilação da RPi4 e conversores DC/DC

#### Baterias (Mensal)
- Verificar tensão das baterias VRLA (deve estar próximo de 12V em repouso)
- Inspecionar terminais (corrosão, oxidação)
- Verificar nível de carga (evitar descarga profunda)
- Equalizar carga se necessário (carregar completamente)

#### Software (Mensal)
- Atualizar pacotes ROS: `sudo apt update && sudo apt upgrade`
- Verificar logs de ROS para erros persistentes: `rosrun rqt_console rqt_console`
- Backup de configurações e mapas
- Verificar espaço em disco: `df -h`

### Problemas Comuns e Soluções

#### LiDAR não detecta obstáculos
- **Causa**: Janela suja ou obstruída
- **Solução**: Limpar janela de vidro, verificar conexão Ethernet
- **Verificação**: `rostopic echo /scan` deve mostrar dados variando

#### Robô não se move
- **Causa**: Bateria descarregada, Pico não responde, driver de motor com problema
- **Solução**: 
  1. Verificar tensão da bateria 2 (deve ser >11V)
  2. Verificar conexão USB RPi4-Pico
  3. Reiniciar Pico (desconectar e reconectar)
  4. Verificar comandos PWM no Pico

#### Expressões faciais não mudam
- **Causa**: ESP32 sem conexão Bluetooth, aplicativo não conectado
- **Solução**:
  1. Verificar se ESP32 está alimentado
  2. Emparelhar tablet com ESP32 (`8C:AA:B5:93:69:EE`)
  3. Reiniciar ESP32
  4. Verificar logs da API REST

#### Aplicativo não conecta à API
- **Causa**: Raspberry Pi 4 não está na rede MrRoboto, API não está rodando
- **Solução**:
  1. Verificar conexão Wi-Fi da RPi4: `iwconfig`
  2. Verificar que API Flask está rodando: `ps aux | grep flask`
  3. Testar conexão: `curl http://192.168.1.100:5000/metadata/version`
  4. Verificar firewall: `sudo ufw status`

#### Odometria deriva (robô perde localização)
- **Causa**: Encoders descalibrados, roda patinando, piso escorregadio
- **Solução**:
  1. Recalibrar odometria (ajustar `wheel_radius` e `wheel_separation`)
  2. Verificar aderência das rodas
  3. Reinicializar localização AMCL no RViz ("2D Pose Estimate")

#### Autonomia menor que o esperado
- **Causa**: Baterias degradadas, uso intensivo de motores/processamento
- **Solução**:
  1. Testar tensão das baterias sob carga
  2. Substituir baterias se tensão menor que 11V após carga completa
  3. Otimizar trajetórias (reduzir rotações e acelerações bruscas)
  4. Reduzir brilho dos LEDs

### Calibração e Ajustes

#### Calibração da Odometria
```bash
# 1. Marcar posição inicial no chão
# 2. Comandar robô para frente 2 metros
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'
# Aguardar 10 segundos
# 3. Medir distância real percorrida
# 4. Ajustar parâmetro wheel_radius em mobile_rob_dev/config/robot_params.yaml
# wheel_radius_ajustado = wheel_radius_atual * (distancia_real / distancia_odom)
```

#### Calibração do LiDAR
```bash
# Verificar offset angular (robô deve estar alinhado com parede)
rostopic echo /scan | grep angle_min
# Ajustar parâmetro se necessário
```

### Armazenamento e Transporte

#### Armazenamento (Quando não em uso)
- Desligar todas as baterias (interruptores individuais)
- Carregar baterias VRLA completamente antes de armazenar (evitar sulfatação)
- Armazenar em local seco e arejado (MDF sensível à umidade)
- Cobrir LiDAR para proteger janela de poeira
- Desconectar baterias se armazenamento >1 mês

#### Transporte
- Segurar pela base inferior (mais resistente)
- Evitar impactos no LiDAR (componente mais frágil)
- Desligar baterias durante transporte
- Proteger tablet (remover se possível)

### Atualizações e Melhorias Futuras

#### Possíveis Upgrades de Hardware
1. **SSD via USB 3.0**: Substituir microSD por SSD para melhor I/O
2. **IMU dedicada**: Adicionar MPU-6050 ou BNO055 para melhor odometria
3. **Câmera RGB-D**: Adicionar Intel RealSense para percepção 3D
4. **Bateria LiPo**: Substituir VRLA por LiPo (mais leve, maior densidade energética)
5. **Estrutura em alumínio**: Substituir MDF por perfis de alumínio (mais resistente)
6. **Sonar ultrassônico**: Adicionar sensores US para detecção de obstáculos baixos

#### Melhorias de Software
1. Implementar battery monitoring node (alertas de bateria baixa)
2. Adicionar recovery behaviors customizados
3. Otimizar parâmetros de navegação (DWA, costmaps)
4. Implementar logging automático de métricas
5. Adicionar sistema de telemetria remota

## Considerações de Segurança

### Operação Segura
- ⚠️ **Botão de Emergência**: Sempre acessível e funcional
- ⚠️ **Velocidade Limitada**: Não exceder 0.5 m/s em ambientes com pessoas
- ⚠️ **Supervisão**: Robô deve ser supervisionado durante operação
- ⚠️ **Obstáculos**: Remover obstáculos baixos (não detectados pelo LiDAR 2D)

### Segurança Elétrica
- 🔋 **Baterias VRLA**: Não abrir ou perfurar (contém ácido selado)
- 🔌 **Desconexão**: Sempre desligar baterias antes de manutenção
- ⚡ **Curto-circuito**: Evitar contato entre terminais das baterias
- 🔥 **Incêndio**: Ter extintor classe C próximo durante operação

### Proteção dos Componentes
- 💧 **Umidade**: Manter robô longe de líquidos (componentes eletrônicos não são à prova d'água)
- 🌡️ **Temperatura**: Operar entre 10°C e 35°C
- 🔊 **LiDAR**: Evitar impactos no sensor (componente caro e frágil)
- 📱 **Tablet**: Proteger de quedas (considerar case ou suporte seguro)

## Recursos Adicionais

### Documentação Técnica
- [Raspberry Pi 4 Datasheet](https://www.raspberrypi.org/documentation/)
- [Raspberry Pi Pico Datasheet](https://datasheets.raspberrypi.org/pico/pico-datasheet.pdf)
- [ESP32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
- [Hokuyo URG Series Manual](https://www.hokuyo-aut.jp/)

### Links Úteis
- 🔗 [ROS 1 Noetic Documentation](http://wiki.ros.org/noetic)
- 🔗 [Repositório GitHub: vivaldini/ROBO_DC](https://github.com/vivaldini/ROBO_DC)
- 🔗 [LARIS - UFSCar](https://site.dc.ufscar.br/laris/)

---

**Última Atualização**: Novembro 2025  
**Versão do Documento**: 2.0  
**Responsável**: Equipe LARIS - UFSCar

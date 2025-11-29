---
sidebar_position: 2
---

# Hardware - 1ª Geração

## Plataforma Robótica

### Descrição Geral
O RobôDC de primeira geração utiliza uma plataforma robótica diferencial customizada, construída para ambientes internos.

### Especificações da Base
- **Modelo**: Plataforma customizada diferencial
- **Dimensões**: 45cm (L) x 35cm (C) x 40cm (A)
- **Peso**: Aproximadamente 8 kg (com bateria)
- **Sistema de Locomoção**: Tração diferencial (2 rodas motrizes + 1 roda boba)
- **Velocidade Máxima**: 0.5 m/s (linear), 1.0 rad/s (angular)
- **Autonomia**: Aproximadamente 2-3 horas de operação contínua

### Computador de Bordo
- **Modelo**: Raspberry Pi 4 Model B (4GB RAM) ou Intel NUC
- **Processador**: ARM Cortex-A72 (RPi4) ou Intel Core i5
- **Memória RAM**: 4GB ou 8GB
- **Armazenamento**: Cartão microSD 64GB (RPi4) ou SSD 256GB (NUC)
- **Sistema Operacional**: Ubuntu 20.04 LTS

## Sensores

### Câmeras

#### Câmera Principal
- **Modelo**: Logitech C920 ou Raspberry Pi Camera Module V2
- **Resolução**: 1920x1080 (Full HD)
- **FPS**: 30 fps
- **Interface**: USB 3.0 (Logitech) / CSI (RPi Camera)
- **Campo de Visão**: 78° (diagonal)
- **Posição no Robô**: Frontal, altura aprox. 30cm do solo

### LIDAR

- **Modelo**: RPLidar A1 ou A2
- **Alcance**: 12 metros (A1) ou 16 metros (A2)
- **Precisão**: ±1% da distância medida
- **Taxa de Varredura**: 5.5 Hz (A1) ou 10 Hz (A2)
- **Resolução Angular**: 1° (360 pontos)
- **Interface**: USB (adaptador serial)
- **Posição no Robô**: Centro superior, altura aprox. 25cm do solo

### IMU (Unidade de Medição Inercial)

- **Modelo**: MPU-6050 ou BNO055
- **Sensores**: Acelerômetro de 3 eixos, Giroscópio de 3 eixos
- **Magnetômetro**: Incluído no BNO055
- **Taxa de Amostragem**: 100 Hz
- **Interface**: I2C
- **Posição no Robô**: Próximo ao centro de massa

### Encoders

- **Tipo**: Ópticos incrementais
- **Resolução**: 1000 pulsos por rotação (PPR)
- **Localização**: Acoplados aos eixos das rodas motrizes

### Outros Sensores

- **Sensor 1**: [Especificações]
- **Sensor 2**: [Especificações]

## Atuadores

### Motores

#### Motores de Tração
- **Modelo**: Motor DC com caixa de redução
- **Tipo**: DC com escovas
- **Tensão**: 12V
- **Corrente Máxima**: 2A por motor
- **Torque**: 5 Nm (com redução)
- **Rotação Máxima**: 100 RPM (com redução)
- **Quantidade**: 2 motores (um para cada roda)

### Drivers de Motor

- **Modelo**: L298N ou IBT-2
- **Tensão de Operação**: 6-12V
- **Corrente Máxima**: 3A por canal
- **Controle**: PWM (Pulse Width Modulation)
- **Interface**: GPIO (sinais digitais e PWM)

## Sistema de Energia

### Bateria Principal

- **Tipo**: LiPo 3S (11.1V nominal)
- **Tensão**: 11.1V (nominal), 12.6V (carregada), 9.0V (mínima)
- **Capacidade**: 5000 mAh
- **Autonomia Estimada**: 2-3 horas de operação
- **Conector**: XT60

### Sistema de Gerenciamento de Energia

- **Reguladores de Tensão**: 
  - Buck converter 12V -> 5V (3A) para Raspberry Pi/sensores
  - Buck converter 12V -> 12V regulado para motores
- **Proteções**: 
  - Sobrecarga: Fusível 10A
  - Curto-circuito: Proteção integrada nos conversores
  - Subtensão: Alarme em 9.5V, desligamento em 9.0V
- **Monitoramento**: 
  - Voltagem da bateria via divisor de tensão e ADC
  - Corrente via sensor INA219 (opcional)

## Conexões Físicas Relevantes

### Diagrama de Conexões

```
Bateria LiPo 3S (11.1V)
    │
    ├── Fusível 10A
    │
    ├── Buck Converter (5V/3A) ───> Raspberry Pi 4
    │                                    ├── USB ─────> Câmera Logitech C920
    │                                    ├── USB ─────> RPLidar A1 (via adaptador)
    │                                    ├── I2C ─────> IMU (MPU-6050)
    │                                    ├── GPIO ────> Driver de Motores (L298N)
    │                                    └── GPIO ────> Encoders (Ópticos)
    │
    └── Buck Converter (12V/5A) ───> Driver de Motores L298N
                                         ├──> Motor Esquerdo
                                         └──> Motor Direito

Monitoramento:
    Bateria ──> Divisor de Tensão ──> GPIO ADC (Raspberry Pi)
```

### Pinagem Raspberry Pi GPIO

| Pino GPIO | Função | Conexão |
|-----------|---------|----------|
| GPIO 17 | PWM Motor Esq | L298N ENA |
| GPIO 27 | Direção Motor Esq | L298N IN1 |
| GPIO 22 | Direção Motor Esq | L298N IN2 |
| GPIO 23 | PWM Motor Dir | L298N ENB |
| GPIO 24 | Direção Motor Dir | L298N IN3 |
| GPIO 25 | Direção Motor Dir | L298N IN4 |
| GPIO 5 | Encoder Esq A | Encoder canal A |
| GPIO 6 | Encoder Esq B | Encoder canal B |
| GPIO 13 | Encoder Dir A | Encoder canal A |
| GPIO 19 | Encoder Dir B | Encoder canal B |

### Conexões I2C

| Dispositivo | Endereço I2C | SDA | SCL |
|-------------|--------------|-----|-----|
| MPU-6050 IMU | 0x68 | GPIO 2 | GPIO 3 |

## Montagem Física

### Layout do Robô

[Incluir imagem ou diagrama mostrando a disposição dos componentes]

### Considerações de Montagem

- **Distribuição de Peso**: [Considerações]
- **Proteção dos Sensores**: [Como os sensores são protegidos]
- **Ventilação**: [Sistema de refrigeração]
- **Acessibilidade**: [Facilidade de manutenção]

## Limitações de Hardware

### Processamento
- **CPU limitada**: Raspberry Pi 4 pode ter dificuldades com processamento pesado de imagem
- **Memória**: 4GB de RAM pode ser limitante para múltiplas tarefas simultâneas

### Sensoriamento
- **LIDAR 2D**: Não detecta obstáculos fora do plano horizontal (ex: mesas, cadeiras)
- **Câmera única**: Falta de percepção de profundidade nativa
- **IMU sem magnetômetro** (MPU-6050): Deriva no heading ao longo do tempo

### Locomoção
- **Tração diferencial**: Dificuldade em superfícies irregulares ou com atrito desigual
- **Altura livre de solo**: Limitada (aprox. 5cm), dificultando superação de obstáculos
- **Velocidade**: Limitada para garantir estabilidade e segurança

### Energia
- **Autonomia**: 2-3 horas pode ser insuficiente para tarefas longas
- **Tempo de recarga**: 1-2 horas com carregador padrão
- **Degradação da bateria**: Performance diminui após ~300 ciclos de carga

### Comunicação
- **WiFi**: Dependência de rede WiFi estável para operação remota
- **Latência**: Pode aumentar com distância do roteador ou interferências

## Manutenção

### Manutenção Regular
- Verificação de conexões
- Limpeza de sensores
- Calibração periódica

### Problemas Comuns
[Lista de problemas conhecidos e soluções]

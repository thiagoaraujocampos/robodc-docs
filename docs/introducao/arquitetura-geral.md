---
sidebar_position: 4
---

# Arquitetura Geral

## Diagrama de Alto Nível

A arquitetura do RobôDC segue o padrão clássico de sistemas robóticos, com camadas de sensoriamento, processamento, controle e atuação:

```
┌─────────────────────────────────────────┐
│         Sistema RobôDC                  │
│                                         │
│  ┌─────────────┐      ┌──────────────┐ │
│  │   Sensores  │──────│ Processamento│ │
│  │ - LIDAR     │      │  - SLAM      │ │
│  │ - Câmeras   │      │  - Navegação │ │
│  │ - IMU       │      │  - Percepção │ │
│  │ - Encoders  │      │  - API       │ │
│  └─────────────┘      └──────────────┘ │
│         │                     │         │
│         └─────────┬───────────┘         │
│                   │                     │
│            ┌──────▼──────┐              │
│            │   Controle  │              │
│            │ - Odometria │              │
│            │ - Vel. Cmd  │              │
│            └──────┬──────┘              │
│                   │                     │
│            ┌──────▼──────┐              │
│            │  Atuadores  │              │
│            │ - Motores   │              │
│            │ - LEDs      │              │
│            └─────────────┘              │
└─────────────────────────────────────────┘
```

## Componentes Principais do Sistema

### Sensores

O subsistema de sensoriamento é responsável por capturar informações do ambiente:

- **LIDAR**: Mapeamento 2D/3D do ambiente, detecção de obstáculos
- **Câmeras**: Visão computacional, reconhecimento de objetos e pessoas
- **IMU** (Inertial Measurement Unit): Orientação, aceleração, velocidade angular
- **Encoders**: Medição de rotação das rodas para odometria
- **Sensor de bateria**: Monitoramento do nível de carga

### Processamento

A camada de processamento executa algoritmos de alto nível:

- **SLAM** (Simultaneous Localization and Mapping): Construção de mapas e localização
- **Navegação**: Planejamento de caminhos e evasão de obstáculos
- **Percepção**: Processamento de imagens, reconhecimento de padrões
- **API REST**: Interface para controle remoto via aplicativo móvel
- **Lógica de comportamento**: Behavior trees e máquinas de estado

### Controle

O subsistema de controle faz a ponte entre planejamento e atuação:

- **Cálculo de odometria**: Integração de encoders e IMU
- **Controle de velocidade**: Converte comandos de alto nível em velocidades das rodas
- **Transformações (TF)**: Mantém árvore de coordenadas do robô
- **Publicação de estados**: Disponibiliza informações para outros módulos

### Atuadores

Componentes que executam ações físicas:

- **Motores DC**: Locomoção diferencial (roda esquerda/direita)
- **Driver de motores**: Controle PWM via microcontrolador
- **LEDs**: Expressões faciais e feedback visual
- **Alto-falante** (opcional): Feedback sonoro e síntese de voz

## Diferenças Conceituais entre 1ª e 2ª Geração

### 1ª Geração

- Baseada em **ROS 1 Noetic**
- Arquitetura mais simples e monolítica
- Navigation Stack clássico
- Comunicação via roscore centralizado
- Foco em validação de conceitos

### 2ª Geração

- Baseada em **ROS 2 Humble**
- Arquitetura distribuída com DDS
- **Nav2** (Navigation 2) com behavior trees
- Maior modularidade e separação de responsabilidades
- Melhor segurança e confiabilidade
- Suporte a execução em tempo real (com RT patches)

### Tabela Comparativa

| Aspecto | 1ª Geração | 2ª Geração |
|---------|------------|------------|
| Framework ROS | ROS 1 Noetic | ROS 2 Humble |
| Comunicação | roscore (centralizado) | DDS (distribuído) |
| Navegação | Navigation Stack | Nav2 + Behavior Trees |
| Build System | catkin_make | colcon |
| Launch Files | XML | Python |
| Segurança | Sem autenticação | Suporte a DDS Security |
| Tempo Real | Não | Sim (com patches RT) |
| Lifecycle | Não | Sim (Managed Nodes) |

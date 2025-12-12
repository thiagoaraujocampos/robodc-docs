---
sidebar_position: 3
---

# Repositório do Robô de 2ª Geração

## Informações do Repositório

- **Nome**: robodc-2gen
- **Tecnologia Principal**: ROS 2 Humble (Robot Operating System 2)
- **Build System**: colcon
- **Linguagens**: Python, C++

## Papéis Principais

### Objetivo

O repositório da 2ª geração contém a implementação completa do RobôDC usando ROS 2, representando uma evolução significativa em relação à 1ª geração. As principais motivações incluem:

- Migração para ROS 2 Humble para melhor suporte de longo prazo
- Arquitetura mais robusta e distribuída
- Uso de tecnologias modernas (Nav2, Behavior Trees, DDS)
- Melhor modularidade e manutenibilidade
- Suporte a recursos avançados não disponíveis no ROS 1

### Responsabilidades

- Controle avançado do robô com ROS 2
- Navegação autônoma usando Nav2
- Processamento distribuído via DDS
- Implementação de algoritmos modernos de SLAM
- Melhor integração com sensores modernos
- Suporte a lifecycle management
- Behavior trees para lógica de comportamento

## Estrutura Básica

```
ros2_ws/
├── src/
│   └── robodc-2gen/
│       ├── robodc_bringup/       # Launch files principais
│       ├── robodc_description/   # URDF, meshes, modelos
│       ├── robodc_navigation/    # Nav2 e navegação
│       ├── robodc_perception/    # Sensores e percepção
│       ├── robodc_control/       # Controle e drivers
│       ├── robodc_interfaces/    # Msgs, Srvs, Actions
│       ├── robodc_behaviors/     # Behavior trees
│       ├── robodc_simulation/    # Gazebo, mundos
│       └── robodc_common/        # Utilitários compartilhados
├── build/                     # Arquivos de build
├── install/                   # Pacotes instalados
└── log/                       # Logs de compilação
```

Para detalhes sobre cada pacote, consulte a seção [Estrutura do Repositório](../robo-2gen/comecando/estrutura-repositorio.md).

## Status do Repositório

- **Estado Atual**: Em desenvolvimento ativo
- **Versão do ROS**: ROS 2 Humble Hawksbill
- **Sistema Operacional**: Ubuntu 22.04 LTS (Jammy Jellyfish)

## Melhorias em Relação à 1ª Geração

### Arquitetura

- **ROS 2 vs ROS 1**: Comunicação DDS distribuída ao invés de master centralizado
- **Lifecycle nodes**: Melhor controle de inicialização e finalização de componentes
- **Modularidade**: Separação mais clara de responsabilidades entre pacotes

### Navegação

- **Nav2**: Stack de navegação moderna com behavior trees
- **SLAM Toolbox**: SLAM mais robusto que GMapping
- **Planejadores avançados**: Múltiplos planejadores disponíveis (NavFn, SmacPlanner, etc.)

### Desenvolvimento

- **colcon**: Build system mais flexível que catkin
- **Python 3**: Versão moderna de Python
- **Launch files em Python**: Maior flexibilidade e programação

## Links Relacionados

- Consulte a seção [Repositório Principal](organizacao-geral.md) para informações sobre como acessar todos os repositórios via submódulos
- Veja [Visão Geral - 2ª Geração](../robo-2gen/visao-geral.md) para entender as motivações e melhorias
- Consulte [Pré-requisitos](../robo-2gen/comecando/prerequisitos.md) para começar a trabalhar com o repositório

---
sidebar_position: 1
---

# Organização Geral

## Como o Projeto está Dividido em Repositórios

O projeto RobôDC está organizado em **três repositórios principais**, cada um com responsabilidades específicas:

### Estrutura de Repositórios

```
RoboDC (Organização)
├── vivaldini/ROBO_DC               # Pacotes ROS 1 (1ª geração)
├── thiagoaraujocampos/RoboDC       # Aplicativo móvel (Ionic/Angular)
├── Hugo-Souza/RoboDC_api           # API REST (Flask/Python)
└── thiagoaraujocampos/robodc-docs  # Esta documentação
```

## Repositórios Principais

### 1. vivaldini/ROBO_DC
**Repositório**: https://github.com/vivaldini/ROBO_DC

**Responsabilidade**: Código ROS 1 do robô de 1ª geração

**Conteúdo**:
- Pacote `mobile_rob_dev`: Nó principal do robô, comunicação serial, odometria
- Pacote `mobile_rob_dev_sim`: Simulação no Gazebo
- Pacote `envrobotz`: Ambiente e configurações
- Subpasta `api/`: API Flask antiga (código legado, substituída pela RoboDC_api)

**Tecnologias**: ROS 1 Noetic, C++, Python, Gazebo

### 2. thiagoaraujocampos/RoboDC
**Repositório**: https://github.com/thiagoaraujocampos/RoboDC

**Responsabilidade**: Aplicativo móvel interativo para interação com o robô

**Funcionalidades**:
- **Navegação guiada**: Enviar robô para locais específicos do DC
- **Cardápio do RU**: Consulta do menu do Restaurante Universitário
- **Detecção de expressões**: Reconhecimento facial e emoções (Face API)
- **Controle manual**: Joystick virtual e controle por setas
- **Multilíngue**: Suporte para PT-BR e EN-US
- **TTS (Text-to-Speech)**: Feedback por voz

**Tecnologias**: Ionic 6, Angular 15, TypeScript, Capacitor, Face-API.js

**Compatibilidade**: Funciona com robôs de 1ª e 2ª geração

### 3. Hugo-Souza/RoboDC_api
**Repositório**: https://github.com/Hugo-Souza/RoboDC_api

**Responsabilidade**: API REST para controle e comunicação com o robô

**Endpoints**:
- `/ros/goal`: Lista destinos disponíveis
- `/ros/goTo/<location>`: Envia robô para local específico
- `/ros/status`: Retorna status da navegação
- `/ros/cancel`: Cancela objetivo atual
- `/led/changeExpression`: Controle de expressões faciais (LEDs)
- `/metadata/version`: Versão da API

**Tecnologias**: Flask, Flask-RESTX, ROS 1, Python, Bluetooth (para LEDs)

**Versão Atual**: v1.2.3

## Fluxo de Comunicação

```
┌─────────────────┐
│  App Móvel      │ (Ionic/Angular)
│  thiagoaraujo/  │
│  RoboDC         │
└────────┬────────┘
         │ HTTP Requests
         ↓
┌─────────────────┐
│  API REST       │ (Flask)
│  Hugo-Souza/    │
│  RoboDC_api     │
└────────┬────────┘
         │ ROS Topics/Actions
         ↓
┌─────────────────┐
│  Pacotes ROS    │ (ROS 1 Noetic)
│  vivaldini/     │
│  ROBO_DC        │
└────────┬────────┘
         │ Serial/Hardware
         ↓
┌─────────────────┐
│  Robô Físico    │
│  (Hardware)     │
└─────────────────┘
```

## Convenções de Commits

Cada repositório segue Conventional Commits:
- `feat:` Nova funcionalidade
- `fix:` Correção de bug
- `docs:` Mudanças na documentação
- `refactor:` Refatoração de código
- `test:` Adição ou modificação de testes
- `style:` Formatação de código
- `chore:` Tarefas de manutenção

## Colaboradores Principais

- **Prof. Vivaldini** (vivaldini) - Coordenador, LARIS-UFSCar
- **Prof. Roberto Inoue** - LARIS-UFSCar
- **Thiago Araujo Campos** (thiagoaraujocampos) - App Móvel
- **Hugo Souza** (Hugo-Souza) - API
- **Heitor Souza** (souzaitor) - API
- **Bruno Leonel** (Bruno12leonel) - Colaborador
- **Robson Rogério Dutra Pereira** - LARIS-UFSCar

## Licenças

- **vivaldini/ROBO_DC**: MIT License (Robson Rogério Dutra Pereira, 2023)
- **thiagoaraujocampos/RoboDC**: Não especificada
- **Hugo-Souza/RoboDC_api**: MIT License (Hugo Souza, 2023)

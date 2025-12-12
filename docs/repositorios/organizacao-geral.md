---
sidebar_position: 1
---

# Organização Geral

## Como o Projeto está Dividido em Repositórios

O projeto RobôDC está organizado em **repositórios modulares**, cada um com responsabilidades específicas. Para facilitar o acesso e a gestão unificada, existe um **repositório principal com submódulos** que agrupa todos os componentes do projeto.

### Repositório Principal (com Submódulos)

**Repositório**: https://github.com/thiagoaraujocampos/RoboDC

**Objetivo**: Centralizar todos os componentes do projeto RobôDC em um único local, utilizando **Git Submodules** para referenciar os repositórios individuais. Esta organização permite:

- **Acesso unificado**: Clone todos os componentes do projeto com um único comando
- **Versionamento coordenado**: Gerenciar versões específicas de cada submódulo de forma integrada
- **Facilitar colaboração**: Novos desenvolvedores podem configurar todo o ambiente rapidamente
- **Manter modularidade**: Cada submódulo continua sendo um repositório independente com seu próprio histórico

### Estrutura de Repositórios

```
thiagoaraujocampos/RoboDC (Repositório Principal - Submódulos)
├── ROBO_DC/                       # Submódulo: Pacotes ROS 1 (1ª geração)
├── RoboDC_app/                    # Submódulo: Aplicativo móvel (Ionic/Angular)
├── RoboDC_api/                    # Submódulo: API REST (Flask/Python)
└── robodc-docs/                   # Submódulo: Esta documentação

Repositórios individuais:
├── vivaldini/ROBO_DC              # Pacotes ROS 1 (1ª geração)
├── thiagoaraujocampos/RoboDC_app  # Aplicativo móvel (Ionic/Angular)
├── Hugo-Souza/RoboDC_api          # API REST (Flask/Python)
└── thiagoaraujocampos/robodc-docs # Esta documentação
```

## Repositórios Individuais

Cada componente do projeto possui seu próprio repositório, que pode ser usado independentemente ou como parte do repositório principal (via submódulos).

### 1. vivaldini/ROBO_DC
**Repositório**: https://github.com/vivaldini/ROBO_DC

**Responsabilidade**: Código ROS 1 do robô de 1ª geração

**Conteúdo**:
- Pacote `mobile_rob_dev`: Nó principal do robô, comunicação serial, odometria
- Pacote `mobile_rob_dev_sim`: Simulação no Gazebo
- Pacote `envrobotz`: Ambiente e configurações
- Subpasta `api/`: API Flask antiga (código legado, substituída pela RoboDC_api)

**Tecnologias**: ROS 1 Noetic, C++, Python, Gazebo

### 2. thiagoaraujocampos/RoboDC_app
**Repositório**: https://github.com/thiagoaraujocampos/RoboDC_app

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

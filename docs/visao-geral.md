---
sidebar_position: 2
---

import useBaseUrl from '@docusaurus/useBaseUrl';

# Visão Geral do Projeto

Esta página apresenta uma visão consolidada do **projeto RobôDC** enquanto iniciativa do Departamento de Computação (DC) da UFSCar: seus princípios de concepção, arquitetura geral, gerações de robôs e a organização dos repositórios de código.

A ideia é responder à pergunta: **“O que é o RobôDC como projeto de longo prazo?”**, indo além de um único robô físico ou de uma única configuração.

## Arquitetura Geral do RobôDC

O RobôDC é pensado como uma **plataforma robótica móvel** composta por:

- uma **base física** (hardware de locomoção, sensores, atuadores);
- uma **camada de software** (pilha de controle, integração de sensores, nós ROS);
- uma **camada de projeto** (padrões, documentação, repositórios e processos de desenvolvimento).

Essas camadas são organizadas para permitir que diferentes grupos possam:

- reutilizar a mesma base física;
- substituir ou evoluir componentes de hardware e software;
- documentar e compartilhar o que foi feito, para que outros possam continuar o trabalho.

<img
  src={useBaseUrl('/img/arqprojeto.png')}
  alt="Arquitetura geral em camadas do projeto RobôDC"
  style={{ width: '25%', display: 'block', margin: '2rem auto' }}
/>

## Abordagem Interdisciplinar, Modular e Anfitriã

### Interdisciplinar

O RobôDC foi concebido para ser utilizado em diferentes contextos:

- disciplinas de robótica, sistemas embarcados, controle, IA, visão computacional, etc.;
- uso como anfitrião;
- projetos de pesquisa que necessitam de uma plataforma móvel;
- iniciativas de extensão que levem a robótica para outros públicos.

Isso significa que o projeto precisa se manter **compreensível** para pessoas com formações e interesses distintos, o que motiva uma documentação clara e uma arquitetura organizada.

### Modular

A modularidade é um dos pilares do RobôDC:

- **No hardware:** sensores, motores, fontes de energia e estruturas mecânicas podem ser substituídos ou reorganizados conforme a geração e o experimento.
- **No software:** nós e pacotes são estruturados para que possam ser ativados, desativados ou substituídos sem reescrever todo o sistema.
- **Na documentação:** as gerações e componentes são descritos de forma organizada, permitindo que novas contribuições sejam facilmente integradas.

### Anfitriã

Chamar o RobôDC de **plataforma anfitriã** significa que:

- outros projetos podem “rodar em cima” do RobôDC, reutilizando sua base de hardware e software;
- novos módulos podem ser plugados sem a necessidade de criar um robô completamente novo;
- o projeto pode evoluir em **gerações**, mas mantendo um histórico de decisões, soluções e componentes reaproveitáveis.

## Gerações do RobôDC

O projeto RobôDC é organizado em **gerações**, que representam grandes marcos de evolução do robô.

Atualmente, a documentação cobre principalmente:

- **Robô de 1ª Geração**  
  - Primeira materialização da plataforma.  
  - Foco em validar conceitos de base: locomoção, sensoriamento, arquitetura de software e infraestrutura de documentação.
- **Robô de 2ª Geração**  
  - Evolução da 1ª geração, incorporando melhorias de hardware, organização interna e arquitetura de software.
  - Ajustes motivados por experiências práticas e necessidades identificadas nas atividades com a 1ª geração.

<img
  src={useBaseUrl('/img/basic_timeline.png')}
  alt="Linha do tempo das gerações do RobôDC"
  style={{ width: '100%', display: 'block', margin: '4rem auto' }}
/>

Cada geração possui:

- uma descrição geral;
- uma seção de hardware;
- uma seção de software, instalação e operação.

Essas informações estão detalhadas nas páginas específicas de cada geração e podem ser consultadas conforme a necessidade de quem está utilizando o robô.

## Organização dos Repositórios

Para apoiar a modularidade e a evolução por gerações, o código do RobôDC é organizado em **repositórios separados**, mas relacionados.

```
RoboDC (Organização)
├── vivaldini/ROBO_DC              # Pacotes ROS 1 (1ª geração)
├── thiagoaraujocampos/RoboDC      # Aplicativo móvel (Ionic/Angular)
├── Hugo-Souza/RoboDC_api          # API REST (Flask/Python)
└── thiagoaraujocampos/robodc-docs # Esta documentação
```

### Repositórios Principais

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

### Repositórios do Robô de 1ª Geração

Os repositórios da 1ª geração concentram:

- arquivos de hardware específicos;
- pacotes de software para a 1ª geração;
- scripts de instalação, configuração e operação;
- documentação mais detalhada, focada apenas nessa geração.

### Repositórios do Robô de 2ª Geração

De forma análoga, os repositórios da 2ª geração inclui:

- ajustes e melhorias de hardware em relação à 1ª geração;
- implementação da pilha de software correspondente à arquitetura dessa geração;
- instruções de instalação e uso específicas;
- documentação complementar específicas da 2ª geração.

## Como Esta Visão se Conecta ao Restante da Documentação

A partir desta página de **Visão Geral do Projeto**, você pode seguir para:

- **Robô de 1ª Geração**  
  para detalhes de hardware, software, instalação e operação da primeira versão do robô.
- **Robô de 2ª Geração**  
  para conhecer a evolução e as melhorias implementadas na segunda versão.
- **Desenvolvimento no Projeto**  
  para entender como contribuir, abrir issues, propor mudanças e manter a coerência da plataforma.

Esta organização busca garantir que o RobôDC seja uma **plataforma viva**, que possa continuar sendo usada e evoluída por novos colaboradores.

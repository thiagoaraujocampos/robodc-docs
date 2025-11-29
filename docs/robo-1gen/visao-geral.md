---
sidebar_position: 1
---

# Visão Geral

## Objetivo da 1ª Geração

A primeira geração do RobôDC foi desenvolvida com o objetivo de criar uma plataforma robótica móvel autônoma baseada em ROS 1 Noetic, capaz de navegar autonomamente em ambientes internos do Departamento de Computação da UFSCar, realizar interação social com usuários através de expressões faciais em LEDs, fornecer informações sobre o cardápio do Restaurante Universitário e guiar visitantes até locais específicos do DC.

O projeto foi concebido como uma ferramenta de pesquisa e desenvolvimento em robótica móvel, servindo como plataforma experimental para estudos em navegação autônoma, localização, mapeamento (SLAM), interação humano-robô e sistemas distribuídos baseados em ROS.

### Características Principais
- **Plataforma**: ROS 1 Noetic Ninjemys (Ubuntu 20.04 LTS)
- **Hardware**: Raspberry Pi 4 (principal), Raspberry Pi Pico (motores), ESP32 (LEDs)
- **Sensoriamento**: LiDAR Hokuyo URG (Ethernet), câmera do tablet (Face-API.js)
- **Locomoção**: Diferencial com dois motores DC controlados por PWM
- **Navegação**: Move Base (planejador global + DWA local planner)
- **Localização**: AMCL (Adaptive Monte Carlo Localization) com mapa pré-existente
- **Mapeamento**: Capacidade de criar mapas 2D via SLAM (GMapping)
- **Interface**: Aplicativo móvel Ionic/Angular em tablet Android
- **Conectividade**: Rede Wi-Fi dedicada (MrRoboto), Bluetooth (ESP32), Ethernet (LiDAR)
- **Energia**: 2x baterias VRLA 12V 7Ah (uma dedicada ao LiDAR, outra ao sistema geral)
- **Estrutura**: Base MDF de dois níveis (leve, customizável, baixo custo)

### Ambiente de Operação
- **Local**: Departamento de Computação (DC) da UFSCar - São Carlos, SP
- **Área de cobertura**: 17 locais cadastrados (laboratórios, auditório, salas, copa, banheiros, etc.)
- **Tipo de ambiente**: Interno, estruturado, pisos planos
- **Mapa**: Mapeamento 2D previamente criado via SLAM

## Conquistas e Resultados

### Implementação Técnica
- ✅ **Stack de navegação ROS 1 completo e funcional**
  - Move Base configurado com DWA local planner
  - Planejador global NavFn (Dijkstra otimizado)
  - Costmaps 2D (local e global) operacionais
- ✅ **Mapeamento 2D bem-sucedido**
  - Criação de mapa do DC via GMapping
  - Resolução de 5 cm por célula
  - Cobertura de todos os corredores e salas principais
- ✅ **Sistema de localização funcional**
  - AMCL configurado e calibrado
  - Localização precisa (±5 cm, ±2°)
  - Recuperação de pose após "kidnapping"
- ✅ **Integração estável entre componentes**
  - Comunicação Raspberry Pi 4 ↔ LiDAR Hokuyo via Ethernet
  - Controle de motores via Raspberry Pi Pico (PWM)
  - Expressões faciais via ESP32 + Bluetooth
  - API REST Flask operacional
- ✅ **Interface de usuário completa**
  - Aplicativo móvel Ionic/Angular funcional
  - 4 funcionalidades principais implementadas (navegação, RU, expressões, controle manual)
  - Comunicação Wi-Fi + Bluetooth estável
- ✅ **Arquitetura distribuída**
  - 3 unidades de processamento (RPi4, Pico, ESP32)
  - Separação de responsabilidades (processamento, controle, interface)
  - Baixo acoplamento entre componentes

### Funcionalidades Implementadas
1. **Navegação Autônoma para 17 Locais**:
   - LE-1 a LE-5 (Laboratórios)
   - Auditório, Maker, PPG-CC4
   - Suporte, Receção, Chefia, Graduação
   - Copa, Lig, Reuniões, Banheiros
   - Home (ponto de partida padrão)
2. **Consulta de Cardápio do RU**:
   - Integração com API externa do RU-UFSCar
   - Exibição de cardápio diário no aplicativo
3. **Expressões Faciais**:
   - 45 expressões diferentes em matrizes de LED
   - Controle via Bluetooth (ESP32)
4. **Controle Manual**:
   - Joystick virtual no aplicativo
   - Comandos de velocidade linear e angular

### Publicações e Documentação
- Código-fonte disponível no GitHub (vivaldini/ROBO_DC)
- Documentação técnica completa (esta documentação)
- Apresentações e demos no DC-UFSCar

## Limitações e Lições Aprendidas

### Limitações Identificadas

#### Limitação 1: Dependência de ROS Master (roscore)
O ROS 1 requer um master centralizado (roscore) que, se falhar, compromete toda a comunicação do sistema. Isso cria um ponto único de falha e dificulta a recuperação automática de falhas.

**Impacto**: Se a Raspberry Pi 4 reiniciar ou o roscore travar, todos os nós perdem comunicação e o robô para de funcionar até reinicialização manual.

#### Limitação 2: Sensoriamento 2D Limitado
O LiDAR Hokuyo 2D (240°, 5.6m) não detecta obstáculos acima ou abaixo do plano de varredura (altura do sensor). Isso limita severamente a percepção do ambiente.

**Impacto**: 
- Mesas, cadeiras, pés de mesas não são detectados
- Objetos elevados (extintores na parede) não são detectados
- Pessoas sentadas podem não ser vistas completamente
- Ângulo traseiro de 120° não tem cobertura

#### Limitação 3: Ausência de IMU Dedicada
Sem uma Unidade de Medição Inercial (IMU) dedicada, a odometria depende apenas dos encoders, levando a deriva de orientação ao longo do tempo, especialmente em pisos escorregadios.

**Impacto**: Localização AMCL precisa ser reinicializada frequentemente quando robô patina ou faz muitas rotações.

#### Limitação 4: Baterias VRLA Pesadas e Lentas
Baterias VRLA 12V 7Ah são pesadas (1.5-2kg cada) e têm recarga lenta (4-8h), impactando autonomia e tempo de inatividade.

**Impacto**: 
- Peso total do robô aumenta (8-10kg)
- Eficiência energética reduzida
- Tempo de recarga longo impede operação contínua
- Degradação após 300-500 ciclos

#### Limitação 5: Estrutura MDF Frágil
MDF é leve e barato, mas sensível à umidade, com resistência mecânica limitada e aparência menos profissional.

**Impacto**: 
- Empenamento em ambientes úmidos
- Rachaduras com impactos
- Necessidade de cuidado no transporte
- Limitação para uso externo

#### Limitação 6: Processamento Limitado (RPi4)
Raspberry Pi 4 (Cortex-A72, 4GB RAM) pode ter dificuldades com processamento simultâneo de SLAM + navegação + visão computacional.

**Impacto**: 
- Taxa de atualização do costmap pode cair
- Latência em comandos de velocidade
- SLAM em tempo real pode ser lento
- Limitação para adicionar mais sensores/processamento

#### Limitação 7: Comunicação HTTP com Latência Variável
API REST via HTTP tem latência variável dependendo da carga de rede Wi-Fi, não sendo adequada para controle em tempo real.

**Impacto**: 
- Comandos do aplicativo podem ter delay de 100-500ms
- Não é adequado para teleoperação em alta velocidade
- Controle manual tem "lag" perceptível

### Lições Aprendidas

#### Lição 1: Importância de Sensores 3D
LiDAR 2D é insuficiente para ambientes com obstáculos de múltiplas alturas. Câmeras RGB-D (como Intel RealSense) ou LiDAR 3D são essenciais para percepção completa.

**Aplicação futura**: Adicionar RealSense D435 para detecção de obstáculos 3D.

#### Lição 2: Fusão Sensorial é Crítica
Odometria de encoders sozinha deriva muito. IMU é essencial para estimativa de orientação confiável, especialmente com EKF (Extended Kalman Filter).

**Aplicação futura**: Integrar IMU MPU-6050 ou BNO055 com robot_localization EKF.

#### Lição 3: Arquitetura Distribuída Funciona
Separar responsabilidades em múltiplos processadores (RPi4 para ROS, Pico para motores, ESP32 para LEDs) permite especialização e reduz sobrecarga.

**Aplicação futura**: Manter arquitetura distribuída, mas melhorar comunicação entre unidades.

#### Lição 4: Interface de Usuário é Fundamental
Aplicativo móvel intuitivo é essencial para adoção. Usuários não-técnicos conseguiram usar o robô sem treinamento, graças à interface simples do app Ionic/Angular.

**Aplicação futura**: Continuar investindo em UX/UI, adicionar feedback visual e sonoro.

#### Lição 5: Documentação e Manutenibilidade
Código bem documentado e estruturado (pacotes ROS modulares) facilita manutenção e expansão por novos desenvolvedores.

**Aplicação futura**: Manter padrões de código, usar ferramentas de CI/CD, versionamento semântico.

#### Lição 6: Testes em Ambiente Real são Insubstituíveis
Simulação (Gazebo) ajuda, mas comportamento real (derrapagem, ruído de sensores, interferência Wi-Fi) só é descoberto em testes práticos.

**Aplicação futura**: Estabelecer protocolo de testes em ambiente controlado antes de deploy.

## Motivação para a 2ª Geração

Com base nas limitações e lições aprendidas durante o desenvolvimento e operação da 1ª geração, foram identificadas as seguintes necessidades que motivaram o desenvolvimento da 2ª geração do RobôDC:

### Necessidades Tecnológicas
1. **Migração para ROS 2**:
   - Eliminar dependência de roscore (DDS distribuído)
   - Melhor suporte para sistemas em tempo real (DDS QoS)
   - Arquitetura mais robusta para multi-robôs
   - Melhor integração com sistemas modernos

2. **Sensoriamento 3D**:
   - Adicionar câmeras RGB-D ou LiDAR 3D
   - Detecção de obstáculos em múltiplas alturas
   - Melhor percepção de pessoas (altura completa)
   - Suporte para navegação social (evitar pessoas)

3. **Fusão Sensorial Avançada**:
   - Integrar IMU dedicada (BNO055 ou superior)
   - Usar EKF/UKF para fusão de odometria + IMU + visão
   - Reduzir deriva de orientação
   - Estimativa de estado mais confiável

4. **Hardware Mais Potente**:
   - Processador mais rápido (considerar Jetson Nano/Xavier)
   - Mais RAM (mínimo 8GB)
   - SSD ao invés de microSD
   - GPU para visão computacional (detecção de objetos, people tracking)

5. **Energia Otimizada**:
   - Substituir VRLA por baterias LiPo (mais leves, recarga rápida)
   - Sistema de gerenciamento de bateria (BMS) inteligente
   - Monitoramento de consumo em tempo real
   - Maior autonomia (4-6 horas)

6. **Estrutura Mais Robusta**:
   - Substituir MDF por alumínio ou plástico ABS
   - Design modular para fácil manutenção
   - Resistência a impactos
   - Estética mais profissional

### Necessidades Funcionais
1. **Navegação Social**:
   - Detectar e rastrear pessoas
   - Planejamento de trajetória considerando pessoas (Social Force Model)
   - Velocidade adaptativa (lenta perto de pessoas)
   - Comunicação de intenções (LEDs, som)

2. **Interação Multimodal**:
   - Reconhecimento de voz (speech-to-text)
   - Síntese de fala (text-to-speech) aprimorada
   - Gestos e expressões faciais mais ricas
   - Tela touchscreen (além de LEDs)

3. **Autonomia Completa**:
   - Voltar automaticamente para estação de recarga
   - Detecção de bateria baixa com comportamento automático
   - Recuperação de falhas sem intervenção humana
   - Logging e telemetria para diagnóstico remoto

4. **Integração com Sistemas Institucionais**:
   - Acesso a calendário acadêmico (horários de aulas, eventos)
   - Integração com sistema de reservas de salas
   - Notificações de eventos em tempo real
   - Dashboard web para monitoramento

### Objetivos de Pesquisa
1. **Navegação em Ambientes Dinâmicos**:
   - Algoritmos de navegação social (TEB, DWB)
   - Predição de trajetória de pessoas
   - Planejamento de trajetória em tempo real
   - Avoidance de múltiplos obstáculos dinâmicos

2. **Interação Humano-Robô (HRI)**:
   - Estudo de aceitação de robôs sociais em ambientes acadêmicos
   - Avaliação de expressões faciais e comunicação
   - Análise de padrões de uso do robô
   - Métricas de satisfação de usuários

3. **Sistemas Multi-Robô**:
   - Coordenação entre múltiplos RobôsDC
   - Compartilhamento de mapas e localização
   - Task allocation (distribuição de tarefas)
   - Comunicação robô-robô (peer-to-peer)

4. **Visão Computacional**:
   - Detecção e reconhecimento de objetos
   - People tracking em tempo real
   - Reconhecimento de ambientes (place recognition)
   - SLAM visual (vSLAM)

---

**Conclusão**: A 1ª geração do RobôDC cumpriu seu objetivo de criar uma plataforma funcional de navegação autônoma e interação social. As limitações identificadas e lições aprendidas forneceram a base para o desenvolvimento da 2ª geração, que busca superar essas limitações com hardware mais potente, sensoriamento 3D, ROS 2, navegação social avançada e maior autonomia.

# RELATÓRIO DETALHADO DE HARDWARE - ROBÔ DC 1ª GERAÇÃO

Documento baseado no PDF "Descrição de Arquitetura - RoboDC.pdf"  
Data de Análise: 29 de novembro de 2024

---

## 1. COMPUTADOR DE BORDO

### 1.1 Raspberry Pi 4 Model B
- **Função**: Central de processamento principal
- **Responsabilidades**:
  - Executar o ROS (Robot Operating System)
  - Executar a API REST em Python
  - Gerenciar comunicações entre todos os módulos
  - Atuar como gateway NAT para fornecer internet aos dispositivos conectados
- **Conectividade**:
  - Interface LAN (Ethernet) via roteador
  - Wi-Fi (wlan0) conectado a rede oculta para acesso à internet
  - USB para comunicação com Raspberry Pi Pico
- **Sistema de Refrigeração**: Cooler dedicado
- **Alimentação**: Conectada diretamente a uma das baterias (sem passar por interruptor)
- **Observação**: Inicia automaticamente quando a bateria geral é conectada

---

## 2. MICROCONTROLADORES

### 2.1 Raspberry Pi Pico
- **Função**: Unidade de processamento embarcada para controle de locomoção
- **Responsabilidades**:
  - Interpretar comandos de movimentação
  - Controlar motores de passo
  - Gerar sinais PWM (Pulse Width Modulation) para controle de velocidade
  - Interface direta com drivers de motor
- **Conexão**: USB com Raspberry Pi 4
- **Tipo de Controle**: Controle independente das rodas direita e esquerda

### 2.2 ESP32
- **Função**: Gerenciamento da matriz de LEDs para face digital do robô
- **Responsabilidades**:
  - Controlar Matrizes de LED
  - Exibir expressões faciais digitais
  - Fornecer feedback visual em tempo real
- **Conexão**: Bluetooth com Raspberry Pi 4
- **Proteção**: Revestida por cápsula na haste
- **Observação**: Fios de conexão com as Matrizes de LED ainda expostos

---

## 3. SENSORES

### 3.1 LiDAR Hokuyo (Sensor Laser)
- **Tipo**: LiDAR (Light Detection and Ranging) de varredura a laser
- **Função**: Mapeamento e navegação autônoma
- **Tecnologia**: 
  - Emite pulsos de luz laser
  - Mede tempo de retorno dos pulsos refletidos
  - Gera nuvem de pontos representando obstáculos
- **Conexão**: RJ45 (Cabo Ethernet) via roteador dedicado
- **Alimentação**: Bateria dedicada exclusiva de 12V 7Ah
- **Controle**: Interruptor liga/desliga dedicado
- **Requisitos**: Alimentação estável e contínua

### 3.2 Câmera (via Tablet)
- **Função**: Detecção de faces e reconhecimento de expressões
- **Aplicações**:
  - Reconhecimento facial
  - Identificação de expressões emocionais
  - Ajuste de direção do olhar do robô
- **Integração**: Através do tablet Android

---

## 4. ATUADORES

### 4.1 Motores
- **Tipo**: Motores de passo/corrente contínua
- **Configuração**: Robô móvel diferencial
- **Quantidade**: 2 conjuntos (direito e esquerdo)
- **Controle**: PWM via Raspberry Pi Pico
- **Características**:
  - Controle independente de velocidade
  - Permite curvas, avanço e retrocesso
  - Velocidade ajustável por ciclo de trabalho PWM

### 4.2 Drivers de Motor
- **Função**: Controlar potência enviada aos motores
- **Conexão**: Interface com Raspberry Pi Pico
- **Controle**: Recebem sinais PWM para ajuste de rotação

### 4.3 Matrizes de LEDs
- **Função**: Face digital do robô
- **Propósito**:
  - Exibir expressões emocionais
  - Indicar status do robô
  - Responder a expressões detectadas
- **Controle**: ESP32 via comandos da Raspberry Pi 4
- **Cores**: Múltiplas (baseado em reconhecimento de expressões)

---

## 5. SISTEMA DE ENERGIA

### 5.1 Especificações das Baterias
- **Tipo**: Baterias seladas de chumbo-ácido reguladas por válvula (VRLA)
- **Especificações**: 12V @ 7Ah (cada)
- **Quantidade**: 2 unidades
- **Características**:
  - Confiabilidade elevada
  - Corrente estável
  - Manutenção simplificada

### 5.2 Distribuição de Energia

#### Bateria 1 - Dedicada ao LiDAR
- **Função**: Alimentação exclusiva do sensor Hokuyo
- **Voltagem**: 12V
- **Capacidade**: 7Ah
- **Controle**: Interruptor liga/desliga dedicado
- **Razão**: Evitar interferências elétricas e quedas de tensão

#### Bateria 2 - Uso Geral
- **Função**: Alimentação dos demais componentes
- **Componentes Alimentados**:
  - Raspberry Pi 4
  - Motores e drivers
  - Roteador
  - Cooler da Raspberry Pi 4
- **Voltagem**: 12V
- **Capacidade**: 7Ah
- **Controle**: Interruptor liga/desliga
- **Observação**: Conexão direta com Raspberry Pi 4 (sem passar por interruptor)

### 5.3 Conversores DC/DC Step Down
- **Função**: Redução de tensão para alimentação correta dos componentes
- **Tipo**: Step Down (redutores)
- **Aplicação**: Converter 12V das baterias para tensões necessárias (5V, 3.3V, etc.)
- **Importância**: Garantir alimentação correta após saída das baterias
- **Quantidade**: Múltiplos conversores no circuito

### 5.4 Proteções e Controles

#### Botão de Emergência
- **Nome**: Botão de interrompimento da locomoção
- **Localização**: Nível superior da estrutura
- **Função**: Trava mecânica das rodas
- **Operação**:
  - Apertar: Trava acionada, robô impedido de se locomover
  - Girar: Destrava e restaura movimentação
- **Tipo**: Botão de emergência para parada imediata

#### Interruptores de Bateria
- **Quantidade**: 2 interruptores (um para cada bateria)
- **Interruptor 1**: Liga/desliga LiDAR Hokuyo
- **Interruptor 2**: Liga/desliga cooler da Raspberry Pi 4 e motores
- **Observação**: Raspberry Pi 4 liga automaticamente ao conectar bateria geral

---

## 6. CONECTIVIDADE E COMUNICAÇÃO

### 6.1 Roteador Dedicado
- **Função**: Criar rede local (MrRoboto)
- **Conectividade**:
  - Interface LAN para sensor Hokuyo (RJ45)
  - Interface LAN para Raspberry Pi 4 (Ethernet)
  - Wi-Fi para tablet e aplicativos
- **Propósito**:
  - Comunicação interna entre componentes
  - Acesso à API REST
  - Isolamento da rede interna

### 6.2 Protocolos de Comunicação

#### Entre Componentes Internos:
- **Raspberry Pi 4 ↔ Hokuyo**: Ethernet (RJ45) via roteador
- **Raspberry Pi 4 ↔ ESP32**: Bluetooth
- **Raspberry Pi 4 ↔ Raspberry Pi Pico**: USB
- **Raspberry Pi 4 ↔ Roteador**: Ethernet (LAN)
- **Tablet ↔ Roteador**: Wi-Fi
- **Raspberry Pi 4 ↔ Internet**: Wi-Fi (wlan0) via rede oculta

#### Configuração NAT:
- **IP Forwarding**: Habilitado
- **Interface Externa**: wlan0 (conexão com internet)
- **Função**: Gateway para dispositivos da rede interna
- **Masquerade**: Substituição de IPs de origem
- **Resultado**: Rede interna isolada com acesso à internet

### 6.3 Interfaces de Software
- **ROS (Robot Operating System)**: Event broker para periféricos robóticos
- **API REST**: Interface HTTP para comunicação com aplicativos
- **Web Socket (rosbridge)**: Comunicação em tempo real
- **Bluetooth**: Comunicação com ESP32

---

## 7. DIMENSÕES FÍSICAS E ESTRUTURA

### 7.1 Estrutura Física
- **Material Base**: MDF de 10mm
- **Configuração**: Dois níveis

#### Nível Inferior:
- **Função**: Acomodação de componentes principais
- **Conteúdo**:
  - Raspberry Pi 4
  - Raspberry Pi Pico
  - Baterias (2x 12V 7Ah)
  - Drivers de motor
  - Conversores DC/DC
  - Roteador
  - Cabos e conexões

#### Nível Superior:
- **Função**: Proteção e suporte
- **Características**:
  - Apoio da haste
  - Botão de trava dos motores (emergência)
  - Proteção dos componentes inferiores

### 7.2 Componentes Externos
- **Haste**: Suporte para ESP32 e Matrizes de LED
- **Face Digital**: Matrizes de LED na parte superior
- **Rodas**: 2 conjuntos (direita e esquerda)
- **Sensor Hokuyo**: Montado para varredura horizontal
- **Tablet**: Interface de usuário montada na frente

### 7.3 Observações sobre Construção
- **Estado Atual**: Construção física simplificada
- **Enfoque**: Viabilização inicial do robô
- **Organização de Cabos**: Necessita melhoria (desorganizada)
- **Acessibilidade**: Dificuldade para manutenção e diagnóstico
- **Fiação Exposta**: Fios das Matrizes de LED ainda expostos

---

## 8. INTERFACE DE USUÁRIO (TABLET)

### 8.1 Especificações
- **Plataforma**: Android
- **Aplicativo**: "RobôDC"
- **Desenvolvimento**: Disciplina Interação Humano-Computador (2022)
- **Conexão**: Wi-Fi com rede MrRoboto

### 8.2 Funcionalidades Disponíveis
1. **Navegação e Movimentação**
   - Seleção de locais no mapa do departamento
   - Acompanhamento autônomo até locais
   - Controle remoto manual
   - Ajuste de velocidade linear e angular
   - Botão de parada de emergência

2. **Informações**
   - Cardápio semanal do Restaurante Universitário
   - História da Universidade e Departamento
   - Informações sobre locais do departamento

3. **Reconhecimento Emocional**
   - Detecção de faces via câmera
   - Identificação de expressões
   - Resposta visual via LEDs
   - Mudança de cor da interface

4. **Configurações**
   - Alteração de idioma
   - Ativação/desativação de áudio (text-to-speech)
   - Configuração de IP da API
   - Configuração de conexão Web Socket

### 8.3 Integração com Hardware
- **Câmera**: Detecção de faces e expressões
- **API REST**: Comandos de movimentação
- **Web Socket (rosbridge)**: Controle remoto direto
- **Feedback Visual**: Sincronização com Matrizes de LED

---

## 9. SOFTWARE E SISTEMAS OPERACIONAIS

### 9.1 Raspberry Pi 4
- **Sistema Base**: Linux (distribuição não especificada)
- **Framework Principal**: ROS (Robot Operating System)
- **API**: REST API em Python
- **Serviços**:
  - rosbridge (Web Socket server)
  - Navegação autônoma
  - Gerenciamento de sensores
  - NAT/Gateway

### 9.2 Raspberry Pi Pico
- **Função**: Firmware para controle de motores
- **Tecnologia**: MicroPython ou C/C++
- **Controle**: PWM para motores

### 9.3 ESP32
- **Função**: Controle de Matrizes de LED
- **Protocolo**: Comunicação Bluetooth com Raspberry Pi 4
- **Biblioteca**: Controle de LED (possivelmente FastLED ou similar)

---

## 10. CARACTERÍSTICAS TÉCNICAS ADICIONAIS

### 10.1 Modelo de Locomoção
- **Tipo**: Robô móvel diferencial
- **Rodas Motoras**: 2 (independentes)
- **Direção**: Controlada pela diferença de velocidade entre rodas
- **Capacidades**:
  - Rotação no próprio eixo
  - Curvas variáveis
  - Movimento linear (frente/trás)

### 10.2 Sistema de Mapeamento
- **Tecnologia**: SLAM (Simultaneous Localization and Mapping)
- **Sensor Principal**: LiDAR Hokuyo
- **Output**: Nuvem de pontos 2D
- **Aplicação**: Navegação autônoma por locais mapeados

### 10.3 Recursos de Segurança
1. **Botão de Emergência Físico**: Trava mecânica das rodas
2. **Botão de Parada na Interface**: Parada por software
3. **Interruptores de Bateria**: Controle individual de energia
4. **Bateria Dedicada para LiDAR**: Evita falhas no sensor crítico
5. **Rede Isolada**: Segurança de comunicação

---

## 11. LIMITAÇÕES E MELHORIAS NECESSÁRIAS

### 11.1 Limitações Identificadas
1. **Organização Física**:
   - Cabos e conexões desorganizados
   - Dificuldade de manutenção
   - Fiação exposta (Matrizes de LED)

2. **Sistema Elétrico**:
   - Raspberry Pi 4 liga automaticamente (sem controle de interruptor)
   - Necessidade de melhor documentação do circuito

3. **Estrutura**:
   - Construção simplificada
   - Material MDF (baixa durabilidade)
   - Necessita proteção adicional

4. **Modularidade**:
   - Dificuldade para adicionar novos componentes
   - Necessidade de reestruturação para expansões

### 11.2 Melhorias Recomendadas pelo Documento
1. **Hardware**:
   - Reorganização de cabos
   - Melhor proteção dos componentes
   - Estrutura mais robusta
   - Adição de sensores modulares

2. **Software**:
   - Melhor documentação da API
   - Documentação dos nós ROS
   - Gerenciamento de pacotes
   - Modularização do código

3. **Procedimentos**:
   - Documentação de manutenção
   - Guias de expansão
   - Procedimentos de diagnóstico

4. **Documentação**:
   - Diagramas elétricos detalhados
   - Esquemáticos de conexão
   - Guias de desenvolvimento

---

## 12. APLICAÇÕES E USO ATUAL

### 12.1 Contexto Acadêmico
- **Disciplinas que utilizam**:
  - Interação Humano-Computador
  - Engenharia de Sistemas
  - Introdução à Programação de Robôs Móveis

### 12.2 Localização
- **Base de Operação**: Espaço Maker - Departamento de Computação (DC/UFSCar)
- **Ambiente de Testes**: Departamento de Computação
- **Função**: Anfitrião e recepcionista educacional

### 12.3 Funcionalidades em Uso
- Navegação autônoma pelo departamento
- Guia para visitantes
- Reconhecimento e resposta emocional
- Apresentação de informações institucionais
- Plataforma de aprendizado interdisciplinar

---

## 13. RESUMO EXECUTIVO DE ESPECIFICAÇÕES

| Categoria | Especificação |
|-----------|---------------|
| **Processador Principal** | Raspberry Pi 4 Model B |
| **Microcontroladores** | Raspberry Pi Pico + ESP32 |
| **Sensor Principal** | LiDAR Hokuyo (varredura laser) |
| **Câmera** | Integrada via tablet Android |
| **Atuadores** | 2x motores de passo (corrente contínua) |
| **Display/Face** | Matrizes de LEDs |
| **Interface Usuário** | Tablet Android |
| **Baterias** | 2x 12V 7Ah VRLA |
| **Conectividade** | Ethernet, Wi-Fi, Bluetooth, USB |
| **Estrutura** | MDF 10mm (dois níveis) |
| **Tipo de Robô** | Móvel diferencial |
| **Software Principal** | ROS (Robot Operating System) |
| **API** | REST API em Python |
| **Rede Local** | Roteador dedicado (MrRoboto) |

---

## 14. CONCLUSÕES

O Robô DC de 1ª geração apresenta uma arquitetura de hardware funcional, porém com enfoque em viabilização inicial. A plataforma utiliza componentes robustos e bem estabelecidos (Raspberry Pi 4, LiDAR Hokuyo, ESP32), garantindo funcionalidades essenciais de navegação autônoma, interação com usuários e reconhecimento emocional.

### Pontos Fortes:
- Arquitetura modular com separação clara de funções
- Uso de tecnologias estabelecidas (ROS, Raspberry Pi)
- Sistema de energia redundante (duas baterias)
- Múltiplas interfaces de comunicação
- Segurança com botão de emergência

### Áreas de Melhoria:
- Organização física e gestão de cabos
- Estrutura mais robusta
- Documentação técnica detalhada
- Modularização para expansões futuras
- Sistema elétrico mais controlado

O robô serve efetivamente como plataforma educacional e de pesquisa, cumprindo seu objetivo de ser um ponto de convergência para formação acadêmica interdisciplinar no contexto da UFSCar.

---

**Documento gerado a partir de**: "Descrição de Arquitetura - RoboDC.pdf"  
**Autor do PDF Original**: Thiago Araujo Campos  
**Versão do Documento Original**: 1.0 (29/11/2024)  
**Data desta Análise**: 29 de novembro de 2024

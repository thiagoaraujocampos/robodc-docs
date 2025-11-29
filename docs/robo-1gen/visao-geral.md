---
sidebar_position: 1
---

# Visão Geral - 1ª Geração

## Objetivo da 1ª Geração

A primeira geração do RobôDC foi desenvolvida com o objetivo de criar uma plataforma robótica móvel autônoma baseada em ROS 1, capaz de navegar em ambientes internos estruturados, realizar mapeamento e localização simultânea (SLAM), e servir como base para pesquisas em robótica móvel.

### Características Principais
- Baseado em ROS 1 Noetic Ninjemys
- Plataforma diferencial com dois motores DC
- Sistema de navegação autônoma com Nav Stack
- Capacidade de mapeamento 2D com SLAM
- Sensoriamento por LIDAR 2D e câmera RGB

## Conquistas

- Implementação completa do stack de navegação ROS 1
- Mapeamento bem-sucedido de ambientes internos
- Sistema de localização funcional com AMCL
- Integração estável entre sensores e atuadores
- Base de código documentada e modular

## Limitações e Lições Aprendidas

### Limitações Identificadas

#### Limitação 1: Dependência de Roscore
O ROS 1 requer um master centralizado (roscore) que, se falhar, compromete toda a comunicação do sistema. Isso cria um ponto único de falha.

#### Limitação 2: Comunicação Baseada em TCP
A comunicação TCPROS não garante entrega em tempo determinístico, dificultando aplicações com requisitos de tempo real rígidos.

#### Limitação 3: Sensoriamento 2D Limitado
O LIDAR 2D não detecta obstáculos acima ou abaixo do plano de varredura, limitando a percepção do ambiente.

#### Limitação 4: Escalabilidade
A arquitetura dificulta operação multi-robô devido à necessidade de configuração manual de namespaces e gestão de rede.

### Lições Aprendidas

#### Lição 1
[Descrição do aprendizado]

#### Lição 2
[Descrição do aprendizado]

#### Lição 3
[Descrição do aprendizado]

## Motivação para a 2ª Geração

Com base nas limitações e lições aprendidas, foram identificadas as seguintes necessidades que motivaram o desenvolvimento da 2ª geração:

- [Necessidade 1]
- [Necessidade 2]
- [Necessidade 3]

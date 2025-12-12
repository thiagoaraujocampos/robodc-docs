---
sidebar_position: 3
---

# Escopo e Motivação

## Motivação

O projeto RobôDC nasceu da necessidade de criar uma **plataforma robótica unificada** que pudesse atender múltiplas demandas do Departamento de Computação da UFSCar:

### Necessidades Educacionais

- Falta de uma plataforma robótica completa para ensino de ROS
- Dificuldade de estudantes experimentarem com robôs reais
- Necessidade de integrar teoria e prática em disciplinas
- Demanda por projetos integradores com hardware real

### Necessidades de Pesquisa

- Plataforma comum para validar algoritmos de navegação
- Infraestrutura para pesquisas em robótica móvel
- Base para comparação de diferentes abordagens
- Ambiente real para testes (vs. apenas simulação)

### Necessidades Institucionais

- Robô anfitrião para o Departamento de Computação
- Projeto integradorque conecte diferentes áreas
- Plataforma para atividades de extensão
- Visão de longo prazo com evolução contínua

## Escopo do Projeto

### O que está incluído

#### Hardware

- Base robótica móvel diferencial
- Sensores de navegação (LIDAR, câmeras, IMU)
- Sistema de computação embarcado
- Sistema de alimentação (baterias)
- Estrutura mecânica e integração

#### Software

- Pilha ROS/ROS 2 completa
- Navegação autônoma (Navigation Stack / Nav2)
- SLAM e localização
- Controle de movimento e odometria
- API REST para controle remoto
- Aplicativo móvel para interação

#### Documentação

- Documentação técnica completa
- Guias de instalação e operação
- Tutoriais e exemplos
- Organização de repositórios
- Procedimentos de segurança

### O que está fora do escopo

#### Limitações Operacionais

- Operação em ambientes externos (chuva, terrenos irregulares)
- Navegação em escadas ou rampas íngremes (> 15°)
- Operação 24/7 sem intervenção
- Manipulação de objetos (sem braço robótico)

#### Funcionalidades Não Incluídas

- Reconhecimento avançado de pessoas (apenas detecção básica)
- Processamento de linguagem natural complexo
- Interação social avançada
- Cooperação multi-robô (pode ser adicionado futuramente)

#### Ambientes

- Operação em ambientes não estruturados
- Navegação em grandes áreas externas
- Ambientes com condições extremas de iluminação

## Justificativa Técnica

### Escolha do ROS

**ROS 1 (1ª geração)**:
- Ecossistema maduro e bem documentado
- Grande quantidade de pacotes disponíveis
- Facilidade de aprendizado inicial

**ROS 2 (2ª geração)**:
- Arquitetura mais robusta e escalável
- Melhor segurança e confiabilidade
- Suporte de longo prazo
- Alinhamento com futuro da robótica

### Arquitetura Modular

A modularidade foi escolhida para:
- Facilitar manutenção e atualizações
- Permitir evolução incremental
- Facilitar colaboração entre diferentes grupos
- Reutilizar componentes entre gerações

### Documentação como Prioridade

- Garantir continuidade do projeto ao longo dos anos
- Facilitar integração de novos colaboradores
- Preservar conhecimento e decisões de projeto
- Servir como referência educacional

## Impacto Esperado

### No Ensino

- Disciplinas mais práticas e engajadoras
- Estudantes com experiência em sistemas robóticos reais
- Aumento no interesse por robótica e IA
- Projetos de graduação e pós-graduação de maior qualidade

### Na Pesquisa

- Facilitar publicações com validação experimental
- Base comum para comparação de resultados
- Plataforma para projetos de pesquisa de longo prazo
- Colaborações com outras instituições

### Na Instituição

- Melhoria da experiência de visitantes no DC
- Visão de inovação e tecnologia
- Integração entre diferentes áreas do departamento
- Projeto demonstrativo para eventos e atividades de extensão

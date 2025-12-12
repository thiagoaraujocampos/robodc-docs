---
sidebar_position: 1
---

# Metodologia de Experimentação

## Sobre Esta Seção

Esta seção documenta a metodologia utilizada para experimentação e validação do projeto RobôDC.

## Ambiente de Testes

Os testes e experimentos com o RobôDC são realizados principalmente nas dependências do Departamento de Computação (DC) da UFSCar, utilizando:

- Corredores e áreas comuns do DC para testes de navegação
- Laboratórios para desenvolvimento e testes controlados
- Ambiente simulado no Gazebo para validação prévia

## Abordagem de Validação

### 1. Simulação

Antes de testes com o robô físico, novos algoritmos e funcionalidades são validados em ambiente simulado (Gazebo), permitindo:

- Testes rápidos de conceitos
- Identificação precoce de problemas
- Desenvolvimento sem risco ao hardware

### 2. Testes em Ambiente Controlado

Após validação na simulação, testes iniciais são realizados em áreas controladas (laboratórios) para:

- Verificar comportamento em hardware real
- Ajustar parâmetros de controle e navegação
- Garantir segurança antes de operação em ambientes reais

### 3. Testes em Ambiente Real

Com o sistema validado, testes em condições reais de operação são conduzidos para:

- Avaliar desempenho em situações práticas
- Identificar limitações e áreas de melhoria
- Coletar dados para análise e refinação

## Ferramentas de Análise

### Coleta de Dados

- **rosbag**: Gravação de tópicos ROS para análise posterior
- **Logs do sistema**: Registro de eventos e erros
- **RViz/RViz2**: Visualização em tempo real de sensores e navegação

### Análise e Visualização

- **rqt_plot**: Gráficos de variáveis em tempo real
- **PlotJuggler**: Análise visual de dados gravados
- **Python/Jupyter**: Scripts customizados para análise estatística

## Critérios de Aceitação

Para cada funcionalidade desenvolvida, estabelecem-se critérios de aceitação que podem incluir:

- Precisão de localização e navegação
- Tempo de resposta do sistema
- Taxa de sucesso em tarefas específicas
- Comportamento seguro em situações inesperadas

## Iteração e Melhoria Contínua

O projeto adota uma abordagem iterativa:

1. Implementação inicial
2. Testes e validação
3. Análise de resultados
4. Identificação de melhorias
5. Refinação e nova iteração

Esta metodologia permite evolução contínua do sistema, incorporando aprendizados de cada ciclo de testes.

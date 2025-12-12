---
sidebar_position: 4
---

# Limitações Conhecidas

## Limitações da 1ª Geração

### Hardware

- **Capacidade de bateria limitada**: Autonomia de aproximadamente 2-3 horas em operação contínua
- **LIDAR 2D apenas**: Não detecta obstáculos acima ou abaixo do plano de varredura
- **Potencia dos motores**: Dificuldade em rampas mais íngremes ou superfícies com atrito elevado
- **Precisão dos encoders**: Deriva na odometria após longos períodos de operação

### Software

- **ROS 1 descontinuado**: Suporte oficial termina em 2025, dificultando manutenção futura
- **Dependência do roscore**: Ponto único de falha; se o roscore cair, todo o sistema para
- **Navigation Stack legado**: Menos flexível que Nav2, sem behavior trees
- **Sem lifecycle management**: Dificuldade em reiniciar componentes sem derrubar todo o sistema

### Operacionais

- **Sensibilidade à iluminação**: Câmeras podem ter desempenho reduzido em ambientes muito escuros ou com muita luz direta
- **Piso irregular**: Dificuldade em superfícies irregulares ou com desníveis
- **Obstáculos transparentes**: Vidros e superfícies transparentes podem não ser detectados pelo LIDAR
- **Ambientes dinâmicos**: Dificuldade com muitas pessoas se movendo simultaneamente

## Limitações da 2ª Geração

### Em Desenvolvimento

A 2ª geração ainda está em desenvolvimento ativo. Algumas funcionalidades planejadas mas ainda não completamente implementadas:

- **Behavior trees complexos**: BTs avançados para comportamentos adaptativos
- **Fusão de sensores avançada**: Integração otimizada de múltiplos sensores
- **Recuperação automática**: Comportamentos de auto-recuperação em situações de falha
- **Navegação social**: Consideração explícita de convenções sociais na navegação

### Conhecidas

- **Curva de aprendizado ROS 2**: ROS 2 é mais complexo que ROS 1, especialmente para iniciantes
- **Documentacao de alguns pacotes**: Alguns pacotes ROS 2 ainda têm documentação menos completa que equivalentes ROS 1
- **Compatibilidade com hardware legado**: Alguns drivers de sensores ainda não foram portados para ROS 2
- **Performance em hardware limitado**: ROS 2 pode consumir mais recursos que ROS 1

## Limitações Gerais (Ambas Gerações)

### Ambiente de Operação

- **Apenas ambientes internos**: Não projetado para operação externa (chuva, terrenos irregulares)
- **Temperatura**: Operação ideal entre 0°C e 40°C
- **Inclinação máxima**: Rampas com inclinação superior a 15° podem comprometer operação
- **Superfícies**: Melhor desempenho em pisos lisos e regulares

### Segurança

- **Operação supervisionada**: Recomenda-se supervisão humana durante operação autônoma
- **Velocidade limitada**: Velocidade máxima restrita para garantir segurança
- **Detecção de pequenos objetos**: Objetos muito pequenos ou próximos ao chão podem não ser detectados

### Funcionalidades

- **Sem manipulação**: Não possui braço robótico para pegar objetos
- **Interação limitada**: Interação humano-robô ainda básica
- **Processamento local**: Limitações de processamento no computador embarcado

## Workarounds e Soluções Temporárias

### Para Deriva de Odometria

- Usar AMCL (Adaptive Monte Carlo Localization) para corrigir posição com base no mapa
- Recalibrar periodicamente a pose do robô
- Integrar IMU para melhorar precisão da odometria

### Para Bateria Limitada

- Implementar estações de recarga automática (em desenvolvimento)
- Monitorar constantemente nível de bateria e retornar à base quando necessário
- Otimizar rotas para minimizar consumo de energia

### Para Obstáculos Transparentes

- Usar câmeras para detectar vidros e superfícies refletivas
- Marcar áreas com vidro no mapa estático
- Adicionar sensores complementares (ultrassom, infravermelho)

### Para Ambientes Dinâmicos

- Ajustar parâmetros do planejador local para ser mais conservador
- Usar costmap inflation maior
- Reduzir velocidade máxima em áreas com alta movimentação

---

*Esta lista é atualizada conforme novas limitações são descobertas e soluções são desenvolvidas.*

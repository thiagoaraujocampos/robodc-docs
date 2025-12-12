---
sidebar_position: 3
---

# Procedimentos de Segurança

## Regras Gerais de Segurança

### Antes da Operação
- [ ] Área de operação delimitada e sinalizada
- [ ] Pessoas autorizadas cientes da operação
- [ ] Botão de emergência testado e acessível
- [ ] Equipamentos de proteção disponíveis (se necessário)

### Durante a Operação
- [ ] Supervisor presente durante operação autônoma
- [ ] Distância de segurança mantida (mínimo 2m recomendado)
- [ ] Monitoramento contínuo do status do robô
- [ ] Atenção a alertas sonoros/visuais

### Após a Operação
- [ ] Robô desligado completamente antes de manutenção
- [ ] Bateria desconectada se necessário manusear componentes
- [ ] Área de operação limpa e organizada

## Emergências

### Parada de Emergência Imediata

**Quando usar**:
- Perigo iminente de colisão
- Comportamento inesperado do robô
- Pessoa entrando na área de operação
- Qualquer situação de risco

**Como executar**:
1. Pressionar botão físico de emergência (vermelho)
2. OU usar comando de software: `rosservice call /emergency_stop` (ROS 1) ou `ros2 service call /emergency_stop std_srvs/srv/Trigger` (ROS 2)

### Procedimento Pós-Emergência

1. **Avaliar a situação**
2. **Identificar causa**
3. **Corrigir problema**
4. **Resetar botão de emergência**
5. **Testar sistemas**
6. **Retomar operação somente se seguro**

## Limites de Operação Segura

### Ambientais
- **Temperatura**: 0°C a 40°C
- **Umidade**: 20% a 80% (sem condensação)
- **Iluminação**: Mínimo 100 lux para câmera
- **Inclinação do solo**: Máximo 15°

### Operacionais
- **Velocidade máxima**: 0.5 m/s (pode ser ajustada conforme necessário)
- **Peso máximo de carga**: 5 kg
- **Bateria mínima para operação**: 20%
- **Distância mínima de obstáculos**: 30cm

## Manutenção Segura

### Antes de Tocar no Robô

1. **Desligar completamente**
2. **Desconectar bateria**
3. **Aguardar 30 segundos**
4. **Verificar ausência de movimento**

### Durante Manutenção

- Usar ferramentas apropriadas
- Não forçar componentes
- Documentar mudanças
- Testar após qualquer modificação

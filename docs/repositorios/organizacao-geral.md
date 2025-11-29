---
sidebar_position: 1
---

# Organização Geral

## Como o Projeto está Dividido em Repositórios

O projeto RobôDC está organizado em múltiplos repositórios Git, cada um com responsabilidades específicas:

### Estrutura de Repositórios

```
robodc/
├── robodc-1gen/          # Código do robô de 1ª geração (ROS 1)
├── robodc-2gen/          # Código do robô de 2ª geração (ROS 2)
├── robodc-docs/          # Documentação do projeto
└── [outros repos]/       # Repositórios auxiliares
```

## Convenções de Nomes

### Repositórios
- **robodc-1gen**: Repositório da primeira geração do robô
- **robodc-2gen**: Repositório da segunda geração do robô
- **robodc-docs**: Documentação centralizada

### Branches
- `main`: Branch principal com código estável
- `develop`: Branch de desenvolvimento
- `feature/*`: Branches para novas funcionalidades
- `fix/*`: Branches para correções

### Commits
Seguimos o padrão de Conventional Commits:
- `feat:` Nova funcionalidade
- `fix:` Correção de bug
- `docs:` Mudanças na documentação
- `refactor:` Refatoração de código
- `test:` Adição ou modificação de testes

## Fluxo de Trabalho

[Descrever o fluxo de trabalho entre os repositórios]

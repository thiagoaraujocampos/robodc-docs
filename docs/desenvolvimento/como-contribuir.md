---
sidebar_position: 1
---

# Como Contribuir

## Bem-vindo!

Obrigado por seu interesse em contribuir com o projeto RobôDC!

## Pré-requisitos para Contribuir

- Conhecimento de ROS 1 ou ROS 2
- Familiaridade com Git
- Python e/ou C++

## Processo de Contribuição

### 1. Fork e Clone

```bash
# Fork o repositório no GitHub

# Clone seu fork
git clone https://github.com/SEU_USUARIO/robodc-[1gen|2gen].git
cd robodc-[1gen|2gen]

# Adicione o repositório original como upstream
git remote add upstream https://github.com/ORIGINAL/robodc-[1gen|2gen].git
```

### 2. Criar Branch

```bash
# Atualizar main
git checkout main
git pull upstream main

# Criar branch para sua feature
git checkout -b feature/minha-feature
```

### 3. Desenvolver

- Fazer mudanças
- Testar localmente
- Seguir padrões de código

### 4. Commit

```bash
# Adicionar arquivos
git add .

# Commit seguindo Conventional Commits
git commit -m "feat: adiciona nova funcionalidade X"
```

**Tipos de commit**:
- `feat`: Nova funcionalidade
- `fix`: Correção de bug
- `docs`: Mudanças na documentação
- `refactor`: Refatoração de código
- `test`: Adição de testes
- `chore`: Tarefas de manutenção

### 5. Push e Pull Request

```bash
# Push para seu fork
git push origin feature/minha-feature
```

Abra Pull Request no GitHub:
- Descrição clara das mudanças
- Referências a issues relacionadas
- Screenshots se aplicável

## Padrões de Código

### Python
- Seguir PEP 8
- Docstrings para funções públicas
- Type hints quando apropriado

### C++
- Seguir ROS 2 Style Guide
- Comentários Doxygen
- RAII e smart pointers

### ROS
- Nomes de tópicos em snake_case
- Nomes de pacotes em snake_case
- Classes em PascalCase

## Testes

```bash
# ROS 1
catkin_make run_tests

# ROS 2
colcon test
colcon test-result --all
```

## Documentação

- Atualizar README.md se necessário
- Adicionar/atualizar docstrings
- Atualizar documentação em docs/

## Code Review

- Todas as contribuições passam por code review
- Seja receptivo a feedback
- Faça as mudanças solicitadas

## Comunicação

- Issues para bugs e features
- Discussions para perguntas
- Pull Requests para código

## Licença

Ao contribuir, você concorda que suas contribuições serão licenciadas sob a mesma licença do projeto.

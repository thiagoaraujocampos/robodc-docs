# RobôDC - Documentação

Documentação oficial do projeto RobôDC, plataforma robótica móvel do Departamento de Computação da UFSCar.

## Sobre o Projeto

O **RobôDC** é uma plataforma interdisciplinar, modular e anfitriã para experimentação em robótica móvel. Esta documentação faz parte de um ecossistema de repositórios organizados através de submódulos Git.

### Repositório Principal

**URL**: https://github.com/thiagoaraujocampos/RoboDC

O repositório principal utiliza **Git Submodules** para centralizar todos os componentes do projeto:
- Pacotes ROS (1ª e 2ª geração)
- Aplicativo móvel (Ionic/Angular)
- API REST (Flask/Python)
- Esta documentação

Para mais informações sobre a organização dos repositórios, consulte a [documentação completa](https://robodc.dev.br).

---

## Website

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

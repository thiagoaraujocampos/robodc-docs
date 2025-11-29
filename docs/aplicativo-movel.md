---
sidebar_position: 8
---

# Aplicativo Móvel RobôDC

## Informações Gerais

- **Repositório**: https://github.com/thiagoaraujocampos/RoboDC
- **Nome do App**: BoT-LY
- **Plataformas**: Android, iOS, Web (PWA)
- **Tecnologias**: Ionic 6, Angular 15, TypeScript, Capacitor

## Visão Geral

O **RoboDC Mobile App** é um aplicativo multiplataforma desenvolvido em **Ionic/Angular** que permite aos usuários interagir com o robô anfitrião do Departamento de Computação da UFSCar de forma intuitiva e amigável.

:::info Compatibilidade
O aplicativo é compatível com **ambas as gerações** do robô (1ª e 2ª geração), comunicando-se através da API REST.
:::

## Funcionalidades Principais

### 1. 🗺️ Navegação Guiada

Permite enviar o robô para locais específicos do Departamento de Computação.

**Locais Disponíveis**:
- Laboratórios: LE-1, LE-2, LE-3, LE-4, LE-5
- Salas: Suporte, PPG-CC4, Maker
- Locais administrativos: Auditório, Banheiros, Copa, LIG, Reuniões, Chefia, Graduação, Recepção

**Fluxo de Uso**:
1. Usuário seleciona um local da lista
2. Visualiza informações sobre o local (foto + descrição)
3. Visualiza localização no mapa interativo do DC
4. Confirma o comando "Guiar-me até lá"
5. App envia request para `/ros/goTo/{location}`
6. Navega para tela "Ongoing" com status em tempo real
7. Robô anuncia via TTS: "Vou te guiar até o local. Me siga!"

### 2. 🍽️ Cardápio do RU

Consulta do cardápio semanal do Restaurante Universitário da UFSCar.

**Características**:
- Busca dados da API PET-BCC: `https://petbcc.ufscar.br/ru_api/`
- Filtra por campus: São Carlos
- Exibe cardápio completo: Almoço, Jantar, Vegetariano
- Seleção por dia da semana
- Interface visual clara e organizada
- Funciona offline após primeira carga (cache)

### 3. 😊 Detecção de Expressões Faciais

Utiliza inteligência artificial para reconhecer emoções e características faciais em tempo real.

**Tecnologia**: **Face-API.js** (TensorFlow.js)

**Funcionalidades**:
- Detecção de emoções: Feliz, Triste, Bravo, Surpreso, Com medo, Enojado, Neutro
- Reconhecimento de gênero
- Estimativa de idade
- Detecção de marcos faciais (landmarks)
- Cor de fundo dinâmica baseada na emoção detectada

**Modelos Carregados**:
```javascript
await faceapi.nets.tinyFaceDetector.loadFromUri('/assets/models');
await faceapi.nets.faceLandmark68Net.loadFromUri('/assets/models');
await faceapi.nets.faceRecognitionNet.loadFromUri('/assets/models');
await faceapi.nets.faceExpressionNet.loadFromUri('/assets/models');
await faceapi.nets.ageGenderNet.loadFromUri('/assets/models');
```

### 4. 🎮 Controle Manual

Permite controlar o robô manualmente através do aplicativo.

**Modos de Controle**:
1. **Setas Direcionais**: Controle clássico com botões de seta
2. **Joystick Virtual**: Controle analógico com stick virtual

**Comandos**:
- Frente / Trás
- Rotação esquerda / direita
- Parar

**Implementação**:
```typescript
emitCmdVel(linear: number, angular: number) {
  const cmdVel = {
    linear: { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular }
  };
  
  this.http.post(`${this.robot_api}/robot/cmd_vel`, cmdVel)
    .subscribe();
}
```

## Arquitetura do App

### Estrutura de Pastas

```
src/
├── app/
│   ├── pages/                    # Páginas da aplicação
│   │   ├── home/                 # Tela inicial
│   │   ├── localizacao/          # Navegação guiada
│   │   ├── ru/                   # Cardápio do RU
│   │   ├── expressions/          # Detecção facial
│   │   ├── controller/           # Controle manual
│   │   ├── ongoing/              # Status de navegação
│   │   ├── config/               # Configurações
│   │   └── chess/                # Easter egg
│   │
│   ├── services/                 # Serviços
│   │   ├── face-api.service.ts   # Face API
│   │   └── tts.service.ts        # Text-to-Speech
│   │
│   ├── models/                   # Tipos TypeScript
│   │   ├── locations.types.ts
│   │   └── ru.types.ts
│   │
│   └── app-routing.module.ts     # Rotas
│
├── assets/                       # Recursos estáticos
│   ├── i18n/                     # Traduções
│   │   ├── en-US.json
│   │   └── pt-BR.json
│   ├── models/                   # Modelos Face-API
│   ├── dc/                       # Fotos dos locais
│   └── flags/                    # Bandeiras (idiomas)
│
└── theme/                        # Estilos globais
    └── variables.scss
```

### Rotas Principais

```typescript
const routes: Routes = [
  { path: 'home', component: HomePage },
  { path: 'localizacao', component: LocalizacaoPage },
  { path: 'ru', component: RuPage },
  { path: 'expressions', component: ExpressionsPage },
  { path: 'controller', component: ControllerPage },
  { path: 'ongoing/:location', component: OngoingPage },
  { path: 'config', component: ConfigPage },
  { path: '', redirectTo: 'home', pathMatch: 'full' }
];
```

## Comunicação com a API

### Configuração

```typescript
// Endereço da API (configurável)
const robot_api = localStorage.getItem('robot_api') || 'http://192.168.1.100:5000';
```

### Endpoints Utilizados

```typescript
// 1. Listar locais disponíveis
GET ${robot_api}/ros/goal
Response: { available_locals: string[] }

// 2. Enviar robô para local
GET ${robot_api}/ros/goTo/{location}
Response: { result: string } // "ACTIVE", "SUCCEEDED", etc.

// 3. Verificar status da navegação
GET ${robot_api}/ros/status
Response: {
  goal_state: string,    // Estado do objetivo
  comm_state: string     // Estado da comunicação
}

// 4. Cancelar navegação
DELETE ${robot_api}/ros/cancel
Response: { result: string }

// 5. Controlar velocidade
POST ${robot_api}/robot/cmd_vel
Body: {
  linear: { x: number, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: number }
}
```

## Internacionalização (i18n)

O app suporta **2 idiomas** usando `@ngx-translate`:

### Português (pt-BR)
```json
{
  "home": {
    "welcome": "Bem-vindo ao",
    "dc": "Departamento de Computação",
    "location": "Localização",
    "ru": "RU",
    "expressions": "Expressões"
  },
  "locations": {
    "guideMessage": "Para onde deseja ser guiado?",
    "followMeTtsMessage": "Vou te guiar até o {{location}}. Me siga!"
  }
}
```

### Inglês (en-US)
```json
{
  "home": {
    "welcome": "Welcome to",
    "dc": "Computer Science Department",
    "location": "Location",
    "ru": "University Restaurant",
    "expressions": "Expressions"
  },
  "locations": {
    "guideMessage": "Where would you like to be guided?",
    "followMeTtsMessage": "I will guide you to {{location}}. Follow me!"
  }
}
```

## Text-to-Speech (TTS)

Serviço de síntese de voz para feedback auditivo.

```typescript
// tts.service.ts
export class TtsService {
  public tssIsOn: boolean = true;

  async speak(text: string) {
    if (!this.tssIsOn) return;
    
    const utterance = new SpeechSynthesisUtterance(text);
    utterance.lang = this.currentLanguage;
    utterance.rate = 1.0;
    
    speechSynthesis.speak(utterance);
  }
}
```

**Exemplos de Uso**:
- "Vou te guiar até o Auditório. Me siga!"
- "Você está feliz!"
- "Homem, aproximadamente 25 anos"

## Serviço Face-API

```typescript
// face-api.service.ts
export class FaceApiService {
  public currentEmotion: string = '';
  public expressionMsg: string = '';
  public ageAndGenderMsg: string = '';
  public expressionColor: string = '#000000';

  async drawLandmarks(video: HTMLVideoElement, canvas: HTMLCanvasElement) {
    const detections = await faceapi
      .detectAllFaces(video, new faceapi.TinyFaceDetectorOptions())
      .withFaceLandmarks()
      .withFaceExpressions()
      .withAgeAndGender();

    // Processar detecções
    if (detections.length > 0) {
      const detection = detections[0];
      
      // Emoção predominante
      const expressions = detection.expressions;
      this.currentEmotion = this.getHighestExpression(expressions);
      
      // Idade e gênero
      this.ageAndGenderMsg = `${detection.gender}, ${Math.round(detection.age)} anos`;
      
      // Cor de fundo baseada na emoção
      this.expressionColor = this.getExpressionColor(this.currentEmotion);
    }
  }
}
```

## Recursos Visuais

### Temas

- **Dark Mode**: Tema escuro padrão
- **Light Mode**: Tema claro alternativo

```scss
// Variáveis de tema (variables.scss)
:root {
  --background-color: #1a1a1a;
  --text-color: #ffffff;
  --primary-color: #3880ff;
  --secondary-color: #31b768;
}

.light-theme {
  --background-color: #ffffff;
  --text-color: #000000;
}
```

### Animações

- Ripple effects nos botões
- Transições suaves entre páginas
- Loading spinners
- Feedback visual em ações

## Configurações

Acessível através da tela inicial (múltiplos toques no logo):

```typescript
// config.page.ts
export class ConfigPage {
  public robot_api: string = '';
  public use_virtual_face: boolean = false;

  save() {
    localStorage.setItem('robot_api', this.robot_api);
    localStorage.setItem('use_virtual_face', String(this.use_virtual_face));
  }

  reset() {
    this.robot_api = 'http://192.168.1.100:5000';
    this.use_virtual_face = false;
    this.save();
  }
}
```

## Build e Deploy

### Desenvolvimento

```bash
# Instalar dependências
npm install

# Servir no navegador
ionic serve
# ou
npm start

# URL: http://localhost:8100
```

### Build para Produção

```bash
# Web (PWA)
ionic build --prod

# Android
ionic capacitor build android

# iOS
ionic capacitor build ios
```

### Capacitor Sync

```bash
# Sincronizar código web com plataformas nativas
npx cap sync

# Abrir no Android Studio
npx cap open android

# Abrir no Xcode
npx cap open ios
```

## Dependências Principais

```json
{
  "dependencies": {
    "@angular/core": "^15.0.0",
    "@ionic/angular": "^6.0.0",
    "@ngx-translate/core": "^14.0.0",
    "@ngx-translate/http-loader": "^7.0.0",
    "@capacitor/core": "^4.0.0",
    "@capacitor/app": "^4.0.0",
    "@awesome-cordova-plugins/http": "^6.0.0",
    "face-api.js": "^0.22.2"
  }
}
```

## Testing

```bash
# Unit tests
npm run test

# E2E tests
npm run e2e
```

## Troubleshooting

### Problema: API não responde

**Solução**:
1. Verificar se o robô está ligado e conectado à rede
2. Verificar endereço IP da API nas configurações
3. Testar conectividade: `ping 192.168.1.100`

### Problema: Face-API não carrega modelos

**Solução**:
1. Verificar se os modelos estão em `assets/models/`
2. Verificar permissões de câmera
3. Limpar cache do navegador

### Problema: TTS não funciona

**Solução**:
1. Verificar se TTS está habilitado nas configurações
2. Verificar volume do dispositivo
3. No Android: verificar permissões de áudio

## Colaboradores

- **Thiago Araujo Campos** (thiagoaraujocampos) - Desenvolvedor Principal
- **Hugo Souza** (Hugo-Souza) - Integração com API
- **Heitor Souza** (souzaitor) - Integração com API

## Links Úteis

- 📱 [Repositório no GitHub](https://github.com/thiagoaraujocampos/RoboDC)
- 📚 [Ionic Documentation](https://ionicframework.com/docs)
- 🎨 [Angular Documentation](https://angular.io/docs)
- 🤖 [Face-API.js](https://github.com/justadudewhohacks/face-api.js)

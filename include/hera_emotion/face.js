// ===== util =====
const getElement = (id) => document.getElementById(id);
const easing = { smooth: t => t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t };

// ===== estado global =====
let animacaoFalaAtiva = false;
let intervalFala = null;

let estadoAtual = 'neutra';
let baseHead = { x: 0, y: 0, rot: 0 };

let idleAtivo = true;
let idleRAF = null;

// ===== Configurações Base do Rosto =====
const BASE = {
    pupilaEsqCx: 145, pupilaDirCx: 205,
    pupilaCy: 220,
    palpebrasY: 210, 
    head: { x: 0, y: 0, rot: 0 }
};

// ===== helpers de animação =====
function aplicarTransformacaoCabeca({ x = 0, y = 0, rot = 0 } = {}) {
  const cx = 175, cy = 220;
  const hera = getElement("hera");
  if (!hera) return;
  // Aplica transformações (Flutuação contínua)
  hera.setAttribute("transform", `translate(${x}, ${y}) rotate(${rot}, ${cx}, ${cy})`);
}

// Loop Idle (Respiração + Movimento contínuo)
function loopIdle(ts) {
  if (!idleAtivo) return;

  // Aumentei a intensidade para ser visível na neutra
  const swayX = Math.sin(ts / 2500) * 3.0;       // Balanço lateral
  const swayY = Math.cos(ts / 2000) * 2.5;       // Balanço vertical leve
  const swayRot = Math.sin(ts / 3500) * 2.5;     // Rotação sutil
  const floatY = Math.sin(ts / 2200) * 4.5;      // Respiração (mais visível)

  // bob extra se estiver falando
  const bob = animacaoFalaAtiva ? Math.sin(ts / 120) * 1.5 : 0;

  aplicarTransformacaoCabeca({
    x: baseHead.x + swayX,
    y: baseHead.y + floatY + bob,
    rot: baseHead.rot + swayRot
  });

  idleRAF = requestAnimationFrame(loopIdle);
}
// Inicia o loop imediatamente
idleRAF = requestAnimationFrame(loopIdle);


// ===== Definição das Expressões =====
const EMOCOES = {
    alegria: {
        bocaPath: "M152,285 Q175,305 198,285",
        sobEsq: "M125,190 Q145,185 165,190", sobDir: "M185,190 Q205,185 225,190",
        pupilaY: 218, palpY: 205, bochechaOp: 0.6, head: { x: 0, y: -2, rot: -2 }
    },
    surpresa: {
        bocaPath: "M170,290 Q175,310 180,290",
        sobEsq: "M125,175 Q145,170 165,175", sobDir: "M185,175 Q205,170 225,175",
        pupilaY: 220, palpY: 200, bochechaOp: 0.3, head: { x: 0, y: -5, rot: 0 }
    },
    nojo: {
        bocaPath: "M160,288 Q175,282 190,288",
        sobEsq: "M125,200 Q145,195 165,202", sobDir: "M185,202 Q205,195 225,200",
        pupilaY: 222, palpY: 212, bochechaOp: 0.2, head: { x: -3, y: 2, rot: -4 }
    },
    angustia: {
        bocaPath: "M155,295 Q175,285 195,295",
        sobEsq: "M125,195 Q145,185 165,195", sobDir: "M185,195 Q205,185 225,195",
        pupilaY: 225, palpY: 214, bochechaOp: 0.2, head: { x: 0, y: 4, rot: 3 }
    },
    pensativa: {
        bocaPath: "M160,290 Q175,290 190,290",
        sobEsq: "M125,190 Q145,188 165,192", sobDir: "M185,198 Q205,194 225,196",
        pupilaY: 215, palpY: 210, bochechaOp: 0.3, head: { x: -4, y: 3, rot: -3 }
    },
    neutra: {
        bocaPath: "M155,285 Q175,295 195,285",
        sobEsq: "M125,195 Q145,188 165,195", sobDir: "M185,195 Q205,188 225,195",
        pupilaY: 220, palpY: 210, bochechaOp: 0.3, head: { x: 0, y: 0, rot: 0 }
    }
};

function mudarExpressao(tipo) {
  if (animacaoFalaAtiva) pararFala();
  estadoAtual = tipo;

  const alvo = EMOCOES[tipo] || EMOCOES.neutra;
  baseHead = { ...alvo.head }; // idle usa esta base para oscilar ao redor

  let startTime = null;
  const duracao = 500;

  function animate(time) {
    if (!startTime) startTime = time;
    const t = easing.smooth(Math.min((time - startTime) / duracao, 1));

    // Paths
    getElement("boca").setAttribute("d", alvo.bocaPath);
    getElement("sobrancelha-esq").setAttribute("d", alvo.sobEsq);
    getElement("sobrancelha-dir").setAttribute("d", alvo.sobDir);

    // Pupilas (Interpolação)
    const deltaY = alvo.pupilaY - 220; // 220 é base
    let deltaX = (tipo === 'pensativa') ? 8 : (tipo === 'nojo' ? -5 : 0);
    
    // Move o grupo da pupila
    getElement("pupila-esq").setAttribute("transform", `translate(${deltaX * t}, ${deltaY * t})`);
    getElement("pupila-dir").setAttribute("transform", `translate(${deltaX * t}, ${deltaY * t})`);

    // Pálpebras
    const py = alvo.palpY;
    getElement("palpebra-esq").setAttribute("d", `M123,${py} Q145,${py-8} 167,${py}`);
    getElement("palpebra-dir").setAttribute("d", `M183,${py} Q205,${py-8} 227,${py}`);

    // Bochechas
    getElement("bochecha-esq").setAttribute("opacity", alvo.bochechaOp);
    getElement("bochecha-dir").setAttribute("opacity", alvo.bochechaOp);

    if (t < 1) requestAnimationFrame(animate);
  }
  requestAnimationFrame(animate);
}

function setEmocao(nome) {
  if (!nome) return;
  const n = nome.toLowerCase();

  const mapa = {
    // ALEGRIA
    'alegria': 'alegria', 'felicidade': 'alegria', 'sorriso': 'alegria', 'contente': 'alegria',
    // SURPRESA
    'surpresa': 'surpresa', 'espanto': 'surpresa', 'assombro': 'surpresa',
    // NOJO
    'nojo': 'nojo', 'repugnância': 'nojo', 'desaprovação': 'nojo', 'eca': 'nojo',
    
    // ANGUSTIA 
    'angústia': 'angustia', 
    'angustia': 'angustia', 
    'tristeza': 'angustia', 'choro': 'angustia', 
    'sofrimento': 'angustia', 'medo': 'angustia', 'terror': 'angustia',
    
    // PENSATIVA
    'pensativa': 'pensativa', 'pensativo': 'pensativa', 'confusa': 'pensativa', 
    'antecipação': 'pensativa', 'vigilância': 'pensativa', 'interesse': 'pensativa',
    
    // NEUTRA (Fallback)
    'neutra': 'neutra', 'confiança': 'neutra', 'amor': 'neutra', 'aprovação': 'neutra', 
    'submissão': 'neutra', 'admiração': 'neutra', 'apreensão': 'neutra', 
    'intimidação': 'neutra', 'remorso': 'neutra', 'ira': 'neutra', 
    'irritação': 'neutra', 'agressividade': 'neutra'
  };

  mudarExpressao(mapa[n] || 'neutra');
}

// ===== Fala =====
//function iniciarFala() {
//  if (animacaoFalaAtiva) return;
//  animacaoFalaAtiva = true;

//  const formasBoca = [
//    "M155,285 Q175,295 195,285", // Neutro
//    "M160,288 Q175,305 190,288", // O
//    "M158,285 Q175,280 192,285"  // Achatado
//  ];
//  let i = 0;

//  intervalFala = setInterval(() => {
//    if (!animacaoFalaAtiva) return;
//    getElement("boca").setAttribute("d", formasBoca[i % formasBoca.length]);
//    i++;
//  }, 120);
//}

function pararFala() {
  animacaoFalaAtiva = false;
  if (intervalFala) { clearInterval(intervalFala); intervalFala = null; }
  mudarExpressao(estadoAtual);
}

function piscar() {
  const shutY = 220; // Pálpebra fechada
  getElement("palpebra-esq").setAttribute("d", `M123,${shutY} Q145,${shutY} 167,${shutY}`);
  getElement("palpebra-dir").setAttribute("d", `M183,${shutY} Q205,${shutY} 227,${shutY}`);
  
  setTimeout(() => {
    const py = (EMOCOES[estadoAtual] || EMOCOES.neutra).palpY;
    getElement("palpebra-esq").setAttribute("d", `M123,${py} Q145,${py-8} 167,${py}`);
    getElement("palpebra-dir").setAttribute("d", `M183,${py} Q205,${py-8} 227,${py}`);
  }, 150);
}
setInterval(piscar, Math.random() * 4000 + 3000);

// ===== Inicialização e ROS =====
mudarExpressao('neutra');

let ros = null;

function conectarROS() {
  if (typeof ROSLIB === 'undefined') {
    console.warn('[HERA] ROSLIB não carregado.');
    return;
  }

  const url = (typeof ROSBRIDGE_URL !== 'undefined') ? ROSBRIDGE_URL : 'ws://localhost:9090';

  ros = new ROSLIB.Ros({ url });

  ros.on('connection', () => {
    console.log('[HERA] Conectado ao rosbridge em', url);
  });

  ros.on('error', (error) => {
    console.error('[HERA] Erro de conexão rosbridge:', error);
  });

  ros.on('close', () => {
    console.warn('[HERA] Conexão com rosbridge fechada.');
  });

  const emotionSub = new ROSLIB.Topic({
    ros: ros,
    name: 'hera/emotion',
    messageType: 'std_msgs/String'
  });

  emotionSub.subscribe((msg) => {
    console.log('[HERA] ROS msg:', msg.data);
    setEmocao(msg.data);
  });
}

window.addEventListener('load', () => {
  conectarROS();
});

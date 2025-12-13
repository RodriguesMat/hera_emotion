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

// ===== Base do Rosto =====
const BASE = {
    pupilaEsqCx: 145, pupilaDirCx: 205,
    pupilaCy: 220,
    palpebrasY: 210, 
    head: { x: 0, y: 0, rot: 0 }
};

function aplicarTransformacaoCabeca({ x = 0, y = 0, rot = 0 } = {}) {
  const cx = 175, cy = 220;
  const hera = getElement("hera");
  if (!hera) return;
  hera.setAttribute("transform", `translate(${x}, ${y}) rotate(${rot}, ${cx}, ${cy})`);
}

function loopIdle(ts) {
  if (!idleAtivo) return;

  let headX = 0, headY = 0, headRot = 0;
  let eyeOffsetX = 0, eyeOffsetY = 0; 

  if (estadoAtual === 'navegando') {
    const walkBob = Math.sin(ts / 160) * 5.0; 
    const walkSway = Math.sin(ts / 320) * 1.5; 
    headX = baseHead.x + walkSway;
    headY = baseHead.y + walkBob;
    headRot = baseHead.rot + (walkSway * 0.3);
    eyeOffsetX = Math.sin(ts / 1500) * 6; 

  } else if (estadoAtual === 'procurando') {
    const floatY = Math.sin(ts / 2000) * 2.0;
    headX = baseHead.x;
    headY = baseHead.y + floatY;
    eyeOffsetX = Math.sin(ts / 800) * 18; 

  } else if (estadoAtual === 'desviando') {
    const thinkingSway = Math.sin(ts / 800) * 4.0;
    headX = baseHead.x + thinkingSway;
    headY = baseHead.y + Math.sin(ts / 600) * 2.0;
    headRot = baseHead.rot + (thinkingSway * 0.5);
    eyeOffsetX = Math.sin(ts / 600) * 10; 

  } else if (estadoAtual === 'encontrado') {    
    const tensionBreath = Math.sin(ts / 800) * 1.5;
    
    headX = baseHead.x; 
    headY = baseHead.y + tensionBreath; 
    headRot = baseHead.rot; 

    // Olhos: TRAVADOS no centro (Zero movimento extra).
    eyeOffsetX = 0;

  } else {
    const swayX = Math.sin(ts / 2500) * 3.0;
    const swayRot = Math.sin(ts / 3500) * 2.5;
    const floatY = Math.sin(ts / 2200) * 4.5;
    const bob = animacaoFalaAtiva ? Math.sin(ts / 120) * 1.5 : 0;
    headX = baseHead.x + swayX;
    headY = baseHead.y + floatY + bob;
    headRot = baseHead.rot + swayRot;
  }

  aplicarTransformacaoCabeca({ x: headX, y: headY, rot: headRot });

  // Movimento dos olhos
  if (eyeOffsetX !== 0 || eyeOffsetY !== 0) {
      getElement("pupila-esq").setAttribute("transform", `translate(${eyeOffsetX}, ${eyeOffsetY})`);
      getElement("pupila-dir").setAttribute("transform", `translate(${eyeOffsetX}, ${eyeOffsetY})`);
  } else {
      getElement("pupila-esq").setAttribute("transform", `translate(0, 0)`);
      getElement("pupila-dir").setAttribute("transform", `translate(0, 0)`);
  }

  idleRAF = requestAnimationFrame(loopIdle);
}
idleRAF = requestAnimationFrame(loopIdle);


// ===== Definição das Expressões =====
const EMOCOES = {
    alegria: {
        bocaPath: "M152,285 Q175,305 198,285",
        sobEsq: "M125,190 Q145,185 165,190", 
        sobDir: "M185,190 Q205,185 225,190",
        pupilaY: 218, 
        palpY: 205, 
        bochechaOp: 0.6, 
        head: { x: 0, y: -2, rot: -2 }
    },
    surpresa: {
        bocaPath: "M170,290 Q175,310 180,290",
        sobEsq: "M125,175 Q145,170 165,175", sobDir: "M185,175 Q205,170 225,175",
        pupilaY: 220, palpY: 200, bochechaOp: 0.3, head: { x: 0, y: -5, rot: 0 }
    },
    nojo: {
        bocaPath: "M160,288 Q175,282 190,288",
        sobEsq: "M125,200 Q145,195 165,202", 
        sobDir: "M185,202 Q205,195 225,200",
        pupilaY: 222, 
        palpY: 212, 
        bochechaOp: 0.2, 
        head: { x: -3, y: 2, rot: -4 }
    },
    angustia: {
        bocaPath: "M155,295 Q175,285 195,295",
        sobEsq: "M125,195 Q145,185 165,195", 
        sobDir: "M185,195 Q205,185 225,195",
        pupilaY: 225, 
        palpY: 214, 
        bochechaOp: 0.2, 
        head: { x: 0, y: 4, rot: 3 },
        labioOp: 0
    },
    pensativa: {
        bocaPath: "M160,290 Q175,290 190,290",
        sobEsq: "M125,190 Q145,188 165,192", 
        sobDir: "M185,198 Q205,194 225,196",
        pupilaY: 215, palpY: 210, 
        bochechaOp: 0.3, 
        head: { x: -4, y: 3, rot: -3 }
    },
    neutra: {
        bocaPath: "M155,285 Q175,295 195,285",
        sobEsq: "M125,195 Q145,188 165,195", 
        sobDir: "M185,195 Q205,188 225,195",
        pupilaY: 220, palpY: 210, 
        bochechaOp: 0.3, 
        head: { x: 0, y: 0, rot: 0 }
    },
    procurando: {
        bocaPath: "M160,290 Q175,292 190,290", 
        sobEsq: "M125,194 Q145,196 165,198", 
        sobDir: "M185,198 Q205,196 225,194",
        pupilaY: 220, 
        palpY: 214, 
        bochechaOp: 0.1, 
        head: { x: 0, y: 8, rot: 0 } ,
        labioOp: 0
  },
    encontrado: {
        bocaPath: "M160,292 Q175,294 190,292",
        sobEsq: "M125,200 Q145,200 165,200", 
        sobDir: "M185,200 Q205,200 225,200",
        pupilaY: 220, 
        palpY: 214, 
        bochechaOp: 0.1, 
        head: { x: 0, y: 8, rot: 0 }, 
        labioOp: 0.4
  },
    navegando: {
        bocaPath: "M155,290 Q175,294 195,290", 
        sobEsq: "M125,190 Q145,188 165,190", 
        sobDir: "M185,190 Q205,188 225,190",
        pupilaY: 220, 
        palpY: 210,
        bochechaOp: 0.3, 
        head: { x: 0, y: 0, rot: 0 },
        labioOp: 0
  },
  desviando: {
        bocaPath: "M155,288 Q175,290 195,288",
        sobEsq: "M125,185 Q145,175 165,185", 
        sobDir: "M185,200 Q205,205 225,200",
        pupilaY: 228,
        palpY: 212,   
        bochechaOp: 0.15, 
        head: { x: 0, y: 5, rot: 5 },
        labioOp: 0.35
  },
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

    getElement("boca").setAttribute("d", alvo.bocaPath);
    getElement("sobrancelha-esq").setAttribute("d", alvo.sobEsq);
    getElement("sobrancelha-dir").setAttribute("d", alvo.sobDir);

    const opAlvo = (alvo.labioOp !== undefined) ? alvo.labioOp : 0.6;
    getElement("labio-superior").setAttribute("opacity", opAlvo);

    // Pupilas
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
    'alegria': 'alegria', 'felicidade': 'alegria', 'sorriso': 'alegria', 'contente': 'alegria',
    
    'surpresa': 'surpresa', 'espanto': 'surpresa', 'assombro': 'surpresa',

    'nojo': 'nojo', 'repugnância': 'nojo', 'desaprovação': 'nojo', 'eca': 'nojo',
    
    'angústia': 'angustia', 
    'angustia': 'angustia', 
    'tristeza': 'angustia', 'choro': 'angustia', 
    'sofrimento': 'angustia', 'medo': 'angustia', 'terror': 'angustia',
    
    'pensativa': 'pensativa', 'pensativo': 'pensativa', 'confusa': 'pensativa', 
    'antecipação': 'pensativa', 'vigilância': 'pensativa', 'interesse': 'pensativa',

    
    'procurando': 'procurando', 'buscando': 'procurando', 
    'scanning': 'procurando', 'scan': 'procurando', 'analisando': 'procurando',

    'encontrado': 'encontrado', 'encontrei': 'encontrado', 
    'pegando': 'encontrado', 'agarra': 'encontrado', 
    'determinada': 'encontrado', 'foco': 'encontrado',

    'navegando': 'navegando', 'andando': 'navegando', 
    'movendo': 'navegando', 'caminhando': 'navegando',

    'desviando': 'desviando', 'obstaculo': 'desviando', 
    'recalculando': 'desviando', 'desvio': 'desviando',
    
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

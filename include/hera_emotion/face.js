// ===== util =====
const getElement = (id) => document.getElementById(id);

// ===== estado global =====
let animacaoFalaAtiva = false;
let intervalFala = null;

let estadoAtual = 'neutra';
let baseHead = { x: 0, y: 0, rot: 0 };

let idleAtivo = true;
let idleRAF = null;

// ===== helpers =====
function aplicarTransformacaoCabeca({ x = 0, y = 0, rot = 0 } = {}) {
  const cx = 175, cy = 220;
  const hera = getElement("hera");
  if (!hera) return;
  hera.setAttribute("transform", `translate(${x}, ${y}) rotate(${rot}, ${cx}, ${cy})`);
}

// idle sway + flutuação + head bob
function loopIdle(ts) {
  if (!idleAtivo) return;

  const swayX = Math.sin(ts / 1600) * 1.5;
  const swayY = Math.cos(ts / 1800) * 1.2;
  const swayRot = Math.sin(ts / 2000) * 1.2;

  // flutuação lenta (±5px)
  const floatY = Math.sin(ts / 3000) * 5;

  // bob da fala
  const bob = animacaoFalaAtiva ? Math.sin(ts / 120) * 0.6 : 0;
  const bobY = animacaoFalaAtiva ? Math.cos(ts / 140) * 0.6 : 0;

  aplicarTransformacaoCabeca({
    x: baseHead.x + swayX,
    y: baseHead.y + swayY + floatY + bobY,
    rot: baseHead.rot + swayRot + bob
  });

  idleRAF = requestAnimationFrame(loopIdle);
}
idleRAF = requestAnimationFrame(loopIdle);

// ===== expressões (enxutas) =====
function mudarExpressao(tipo) {
  if (animacaoFalaAtiva) pararFala();
  estadoAtual = tipo;

  const BASE = {
    pupilaEsqCx: 145, pupilaDirCx: 205,
    pupilaEsqCy: 218, pupilaDirCy: 218,
    palpebrasY: 208, bochechaOpacity: 0.4,
    head: { x: 0, y: 0, rot: 0 }
  };

  const E = {
    alegria: {
      bocaPath: "M155,280 Q175,292 195,280",
      sobrancelhaEsqPath: "M130,194 Q145,189 160,194",
      sobrancelhaDirPath: "M190,194 Q205,189 220,194",
      pupilaEsqCx: BASE.pupilaEsqCx, pupilaDirCx: BASE.pupilaDirCx,
      pupilaEsqCy: 220, pupilaDirCy: 220,
      palpebrasY: 206, bochechaOpacity: 0.55,
      head: { x: 0, y: -2, rot: -2 }
    },
    surpresa: {
      bocaPath: "M170,285 Q175,294 180,285",
      sobrancelhaEsqPath: "M130,182 Q145,178 160,182",
      sobrancelhaDirPath: "M190,182 Q205,178 220,182",
      pupilaEsqCx: BASE.pupilaEsqCx, pupilaDirCx: BASE.pupilaDirCx,
      pupilaEsqCy: 218, pupilaDirCy: 218,
      palpebrasY: 202, bochechaOpacity: 0.45,
      head: { x: 0, y: -6, rot: -3 }
    },
    nojo: {
      bocaPath: "M160,279 Q175,276 190,279",
      sobrancelhaEsqPath: "M128,196 Q145,192 162,197",
      sobrancelhaDirPath: "M190,197 Q205,192 220,195",
      pupilaEsqCx: BASE.pupilaEsqCx - 1, pupilaDirCx: BASE.pupilaDirCx - 2,
      pupilaEsqCy: 217, pupilaDirCy: 217,
      palpebrasY: 209, bochechaOpacity: 0.25,
      head: { x: -6, y: 0, rot: -8 }
    },
    angústia: {
      bocaPath: "M156,286 Q175,273 194,286",
      sobrancelhaEsqPath: "M128,204 Q145,198 162,204",
      sobrancelhaDirPath: "M190,204 Q205,198 220,204",
      pupilaEsqCx: BASE.pupilaEsqCx - 1, pupilaDirCx: BASE.pupilaDirCx + 1,
      pupilaEsqCy: 226, pupilaDirCy: 226,
      palpebrasY: 216, bochechaOpacity: 0.22,
      head: { x: -2, y: 5, rot: 4 }
    },
    pensativa: {
      bocaPath: "M160,283 Q175,280 190,284",
      sobrancelhaEsqPath: "M128,191 Q145,186 162,193",
      sobrancelhaDirPath: "M190,196 Q205,191 220,194",
      pupilaEsqCx: BASE.pupilaEsqCx + 2, pupilaDirCx: BASE.pupilaDirCx + 2,
      pupilaEsqCy: 216, pupilaDirCy: 216,
      palpebrasY: 210, bochechaOpacity: 0.32,
      head: { x: -8, y: 3, rot: -6 }
    },
    // NEUTRA = antiga "confiança"
    neutra: {
      bocaPath: "M160,281 Q175,286 190,281",
      sobrancelhaEsqPath: "M130,192 Q145,188 160,192",
      sobrancelhaDirPath: "M190,192 Q205,188 220,192",
      pupilaEsqCx: BASE.pupilaEsqCx, pupilaDirCx: BASE.pupilaDirCx,
      pupilaEsqCy: 218, pupilaDirCy: 218,
      palpebrasY: 207, bochechaOpacity: 0.45,
      head: { x: 1, y: -1, rot: -1 }
    }
  };

  const S = E[tipo] || E.neutra;
  baseHead = { ...S.head }; // idle usa esta base

  let startTime = null;
  const duracao = 800;

  function animate(time) {
    if (!startTime) startTime = time;
    const t = Math.min((time - startTime) / duracao, 1);

    getElement("boca").setAttribute("d", S.bocaPath);
    getElement("sobrancelha-esq").setAttribute("d", S.sobrancelhaEsqPath);
    getElement("sobrancelha-dir").setAttribute("d", S.sobrancelhaDirPath);

    getElement("pupila-esq").setAttribute("cx", S.pupilaEsqCx);
    getElement("pupila-dir").setAttribute("cx", S.pupilaDirCx);
    getElement("pupila-esq").setAttribute("cy", S.pupilaEsqCy);
    getElement("pupila-dir").setAttribute("cy", S.pupilaDirCy);
    getElement("brilho-esq").setAttribute("cx", S.pupilaEsqCx - 3);
    getElement("brilho-dir").setAttribute("cx", S.pupilaDirCx - 3);
    getElement("brilho-esq").setAttribute("cy", S.pupilaEsqCy - 3);
    getElement("brilho-dir").setAttribute("cy", S.pupilaDirCy - 3);

    const y = S.palpebrasY;
    const palpEsqPath = `M125,${y} Q145,${y - 5} 165,${y}`;
    const palpDirPath = `M185,${y} Q205,${y - 5} 225,${y}`;
    getElement("palpebra-esq").setAttribute("d", palpEsqPath);
    getElement("palpebra-dir").setAttribute("d", palpDirPath);

    getElement("bochecha-esq").setAttribute("opacity", S.bochechaOpacity);
    getElement("bochecha-dir").setAttribute("opacity", S.bochechaOpacity);

    if (t < 1) requestAnimationFrame(animate);
  }
  requestAnimationFrame(animate);
}

// ===== mapeamento/aliases (removidas → neutra confiante) =====
function setEmocao(nome) {
  const n = (nome || '').toLowerCase();

  const mapa = {
    alegria: 'alegria',
    surpresa: 'surpresa',
    nojo: 'nojo',
    'angústia': 'angústia',
    pensativa: 'pensativa',
    neutra: 'neutra',

    // tudo abaixo cai em NEUTRA (confiante)
    confiança: 'neutra', amor: 'neutra', aprovação: 'neutra', submissão: 'neutra',
    admiração: 'neutra', assombro: 'neutra',
    medo: 'neutra', apreensão: 'neutra', intimidação: 'neutra', terror: 'neutra',
    tristeza: 'neutra', remorso: 'neutra', pensativo: 'pensativa',
    ira: 'neutra', irritação: 'neutra', agressividade: 'neutra',
    antecipação: 'pensativa', vigilância: 'pensativa', interesse: 'pensativa',
    repugnância: 'nojo', desaprovação: 'nojo'
  };

  mudarExpressao(mapa[n] || 'neutra');
}

// ===== fala =====
function iniciarFala() {
  if (animacaoFalaAtiva) return;
  animacaoFalaAtiva = true;

  const formasBoca = [
    "M155,280 Q175,295 195,280",
    "M160,285 Q175,290 190,285",
    "M165,283 Q175,288 185,283",
    "M158,282 Q175,292 192,282",
    "M162,284 Q175,289 188,284",
    "M155,281 Q175,286 195,281",
    "M150,282 Q175,297 200,282",
    "M168,285 Q175,287 182,285"
  ];
  let i = 0;

  intervalFala = setInterval(() => {
    if (!animacaoFalaAtiva) return;
    getElement("boca").setAttribute("d", formasBoca[i]);
    const variacao = Math.sin(i * 0.5) * 2;
    getElement("labio-superior").setAttribute("ry", 2 + variacao);

    const op = 0.3 + Math.random() * 0.2;
    getElement("bochecha-esq").setAttribute("opacity", op);
    getElement("bochecha-dir").setAttribute("opacity", op);

    i = (i + 1) % formasBoca.length;
  }, 120 + Math.random() * 80);
}

function pararFala() {
  animacaoFalaAtiva = false;
  if (intervalFala) { clearInterval(intervalFala); intervalFala = null; }
  mudarExpressao('neutra');
}

// ===== piscar =====
function piscar() {
  ["olho-esq-base", "olho-dir-base"].forEach(id => {
    const olho = getElement(id);
    const ry = olho.getAttribute("ry");
    olho.setAttribute("ry", "2");
    setTimeout(() => olho.setAttribute("ry", ry), 150);
  });
}
setInterval(piscar, Math.random() * 2000 + 3000);

// ===== inicialização =====
mudarExpressao('neutra');

const ids = [
  'unwinderDeg', 'feedLength', 'accTime', 'decTime', 'feederVmax',
  'dR1', 'dR2', 'dR3', 'dUnw', 'dFeed', 'dBuf'
];

const state = {
  running: false,
  simTime: 0,
  lastTs: 0,
  unwAng: 0,
  feederVelPrev: 0,
  bufferPosPrev: 0,
  webPhase: 0,
  history: [],
};

const inputs = Object.fromEntries(ids.map((id) => [id, document.getElementById(id)]));
const derivedEl = document.getElementById('derived');

const machineCanvas = document.getElementById('machineCanvas');
const mctx = machineCanvas.getContext('2d');
const velChart = document.getElementById('velChart').getContext('2d');
const accChart = document.getElementById('accChart').getContext('2d');
const posChart = document.getElementById('posChart').getContext('2d');

const fmt = (v, unit = '') => `${Number(v).toFixed(2)}${unit}`;

function updateValueLabels() {
  for (const [id, el] of Object.entries(inputs)) document.getElementById(`${id}Val`).textContent = el.value;
}

function getParams() {
  const p = Object.fromEntries(Object.entries(inputs).map(([k, el]) => [k, Number(el.value)]));
  p.unwOmega = p.unwinderDeg * Math.PI / 180;
  p.rUnw = p.dUnw / 2;
  p.unwLinear = p.unwOmega * p.rUnw;
  p.cycleT = Math.max(0.15, p.feedLength / Math.max(1e-6, p.unwLinear));
  p.moveT = p.cycleT * 0.7;
  p.stopT = p.cycleT * 0.3;
  return p;
}

function feederKinematics(t, p) {
  const tc = ((t % p.cycleT) + p.cycleT) % p.cycleT;
  if (tc >= p.moveT) return { pos: p.feedLength, vel: 0, acc: 0, moving: false };

  let ta = p.accTime;
  let td = p.decTime;
  let tv = p.moveT - ta - td;
  let vPeak;
  let triangular = false;

  if (tv >= 0) {
    const denom = tv + 0.5 * (ta + td);
    vPeak = p.feedLength / Math.max(1e-8, denom);
    vPeak = Math.min(vPeak, p.feederVmax);
    const realD = vPeak * denom;
    if (realD < p.feedLength) vPeak = p.feedLength / denom;
  } else {
    triangular = true;
    const sum = ta + td;
    ta = p.moveT * (ta / sum);
    td = p.moveT - ta;
    vPeak = 2 * p.feedLength / Math.max(1e-8, p.moveT);
  }

  const aUp = vPeak / Math.max(1e-8, ta);
  const aDn = vPeak / Math.max(1e-8, td);

  if (!triangular) {
    if (tc < ta) return { pos: 0.5 * aUp * tc * tc, vel: aUp * tc, acc: aUp, moving: true };
    if (tc < ta + tv) {
      const t2 = tc - ta;
      return { pos: 0.5 * aUp * ta * ta + vPeak * t2, vel: vPeak, acc: 0, moving: true };
    }
    const t3 = tc - ta - tv;
    const p1 = 0.5 * aUp * ta * ta + vPeak * tv;
    return { pos: p1 + vPeak * t3 - 0.5 * aDn * t3 * t3, vel: Math.max(0, vPeak - aDn * t3), acc: -aDn, moving: true };
  }

  if (tc < ta) return { pos: 0.5 * aUp * tc * tc, vel: aUp * tc, acc: aUp, moving: true };
  const t2 = tc - ta;
  return {
    pos: 0.5 * aUp * ta * ta + vPeak * t2 - 0.5 * aDn * t2 * t2,
    vel: Math.max(0, vPeak - aDn * t2),
    acc: -aDn,
    moving: true,
  };
}

function pushHistory(sample) {
  state.history.push(sample);
  while (state.history.length && state.history[0].t < state.simTime - 10) state.history.shift();
}

function polyline(points) {
  mctx.beginPath();
  mctx.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i++) mctx.lineTo(points[i].x, points[i].y);
}

function drawRoller(x, y, r, label, angle = 0) {
  const ctx = mctx;
  const grad = ctx.createRadialGradient(x - r * 0.3, y - r * 0.35, r * 0.15, x, y, r);
  grad.addColorStop(0, '#9ce8ff');
  grad.addColorStop(0.55, '#2ca4e8');
  grad.addColorStop(1, '#0e3b84');

  ctx.shadowBlur = 25;
  ctx.shadowColor = 'rgba(50,200,255,0.9)';
  ctx.fillStyle = grad;
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fill();

  ctx.shadowBlur = 0;
  ctx.strokeStyle = '#8ef3ff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(x, y, r * 0.88, 0, Math.PI * 2);
  ctx.stroke();

  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(angle);
  ctx.strokeStyle = 'rgba(170,245,255,0.85)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(r * 0.82, 0);
  ctx.stroke();
  ctx.restore();

  ctx.fillStyle = '#effdff';
  ctx.font = `${Math.max(14, r * 0.34)}px sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  label.split('\n').forEach((line, i, arr) => {
    const dy = (i - (arr.length - 1) / 2) * (r * 0.38);
    ctx.fillText(line, x, y + dy);
  });
}

function drawMachine(p, kin, bufferY, cutterLift, rollerAngle) {
  const ctx = mctx;
  const w = machineCanvas.width;
  const h = machineCanvas.height;
  ctx.clearRect(0, 0, w, h);

  const S = {
    r1: { x: 100, y: 85, r: p.dR1 * 0.35 },
    r2: { x: 420, y: 85, r: p.dR2 * 0.35 },
    r3: { x: 700, y: 85, r: p.dR3 * 0.35 },
    unw: { x: 100, y: 290, r: p.dUnw * 0.35 },
    feed: { x: 900, y: 85, r: p.dFeed * 0.35 },
    buf: { x: 560, y: 290 + bufferY, r: p.dBuf * 0.35 },
  };

  const webPath = [
    { x: S.unw.x - S.unw.r, y: S.unw.y - S.unw.r },
    { x: S.r1.x - S.r1.r, y: S.r1.y - S.r1.r },
    { x: S.r2.x + S.r2.r, y: S.r2.y - S.r2.r },
    { x: S.r2.x + S.r2.r, y: S.buf.y },
    { x: S.r3.x - S.r3.r, y: S.buf.y },
    { x: S.r3.x - S.r3.r, y: S.r3.y - S.r3.r },
    { x: S.feed.x + S.feed.r, y: S.feed.y - S.feed.r },
  ];

  polyline(webPath);
  ctx.lineWidth = 11;
  ctx.strokeStyle = 'rgba(255,35,125,0.12)';
  ctx.shadowColor = 'rgba(255,70,160,0.95)';
  ctx.shadowBlur = 24;
  ctx.stroke();

  polyline(webPath);
  ctx.lineWidth = 4.5;
  ctx.strokeStyle = '#ff4d7f';
  ctx.shadowBlur = 16;
  ctx.stroke();

  if (state.running) {
    polyline(webPath);
    ctx.setLineDash([12, 20]);
    ctx.lineDashOffset = -state.webPhase;
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = '#ffd8e4';
    ctx.shadowBlur = 12;
    ctx.stroke();
    ctx.setLineDash([]);
  }
  ctx.shadowBlur = 0;

  drawRoller(S.r1.x, S.r1.y, S.r1.r, 'Roller1', rollerAngle * 0.9);
  drawRoller(S.r2.x, S.r2.y, S.r2.r, 'Roller2', rollerAngle * 0.88);
  drawRoller(S.r3.x, S.r3.y, S.r3.r, 'Roller3', rollerAngle * 0.92);
  drawRoller(S.unw.x, S.unw.y, S.unw.r, 'Unwin\nder', state.unwAng);
  drawRoller(S.feed.x, S.feed.y, S.feed.r, 'Feeder', rollerAngle * 1.1);
  drawRoller(S.buf.x, S.buf.y, S.buf.r, 'Active\nBuffer', -rollerAngle * 0.75);

  const cutterX = S.feed.x + S.feed.r + 36;
  const topY = 24;
  const down = 66 * cutterLift;
  ctx.shadowBlur = 18;
  ctx.shadowColor = 'rgba(116, 255, 234, 0.75)';
  ctx.fillStyle = '#86fff0';
  ctx.fillRect(cutterX - 10, topY - 12, 34, 10);
  ctx.fillStyle = '#b6fff7';
  ctx.fillRect(cutterX, topY + down, 14, 50);
  ctx.shadowBlur = 0;

  ctx.fillStyle = '#bcefff';
  ctx.font = '14px sans-serif';
  ctx.textAlign = 'left';
  ctx.fillText(`Cycle ${fmt(p.cycleT, 's')} / Move ${fmt(p.moveT, 's')} / Stop ${fmt(p.stopT, 's')}`, 12, h - 40);
  ctx.fillText(`Feeder: ${kin.moving ? '이송중' : '정지'} | Active Buffer: ${fmt(bufferY / 0.35, ' mm')}`, 12, h - 18);
}

function drawChart(ctx, keys, colors, title, yLabel) {
  const W = ctx.canvas.width, H = ctx.canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#020711';
  ctx.fillRect(0, 0, W, H);

  const pad = { l: 44, r: 12, t: 20, b: 28 };
  const x0 = pad.l, y0 = pad.t, w = W - pad.l - pad.r, h = H - pad.t - pad.b;
  ctx.strokeStyle = 'rgba(119,177,232,0.4)';
  ctx.strokeRect(x0, y0, w, h);
  if (state.history.length < 2) return;

  const tMin = state.history[0].t;
  const tMax = state.history[state.history.length - 1].t;
  let yMin = Infinity, yMax = -Infinity;
  for (const s of state.history) {
    for (const k of keys) {
      yMin = Math.min(yMin, s[k]);
      yMax = Math.max(yMax, s[k]);
    }
  }
  if (Math.abs(yMax - yMin) < 1e-9) { yMin -= 1; yMax += 1; }
  const padY = (yMax - yMin) * 0.1;
  yMin -= padY;
  yMax += padY;

  const sx = (t) => x0 + ((t - tMin) / Math.max(1e-6, tMax - tMin)) * w;
  const sy = (v) => y0 + h - ((v - yMin) / (yMax - yMin)) * h;

  ctx.strokeStyle = 'rgba(172,219,255,0.36)';
  ctx.beginPath();
  ctx.moveTo(x0, sy(0));
  ctx.lineTo(x0 + w, sy(0));
  ctx.stroke();

  keys.forEach((k, i) => {
    ctx.shadowBlur = 12;
    ctx.shadowColor = colors[i];
    ctx.strokeStyle = colors[i];
    ctx.lineWidth = 2;
    ctx.beginPath();
    state.history.forEach((s, idx) => {
      const X = sx(s.t), Y = sy(s[k]);
      if (idx === 0) ctx.moveTo(X, Y);
      else ctx.lineTo(X, Y);
    });
    ctx.stroke();
    ctx.shadowBlur = 0;
  });

  ctx.fillStyle = '#ccefff';
  ctx.font = '12px sans-serif';
  ctx.fillText(`${title} (${yLabel})`, x0, 14);
  keys.forEach((k, i) => {
    ctx.fillStyle = colors[i];
    ctx.fillRect(W - 174, 8 + i * 14, 10, 10);
    ctx.fillStyle = '#dbf5ff';
    ctx.fillText(k, W - 160, 17 + i * 14);
  });
}

function updateDerivedText(p, kin, bufferMM, cut) {
  derivedEl.innerHTML = `
    Unwinder 선속도: <b>${fmt(p.unwLinear, ' mm/s')}</b><br>
    Feeder 평균 속도(동기): <b>${fmt(p.feedLength / p.cycleT, ' mm/s')}</b><br>
    Feeder 현재 속도: <b>${fmt(kin.vel, ' mm/s')}</b> / 가속도: <b>${fmt(kin.acc, ' mm/s²')}</b><br>
    Active Buffer 변위(웹 길이 차 보정): <b>${fmt(bufferMM, ' mm')}</b><br>
    Cutter 상태: <b>${cut > 0.5 ? '절단 동작' : '대기'}</b>
  `;
}

function resetState() {
  state.running = false;
  state.simTime = 0;
  state.lastTs = 0;
  state.unwAng = 0;
  state.webPhase = 0;
  state.feederVelPrev = 0;
  state.bufferPosPrev = 0;
  state.history = [];
}

function tick(ts) {
  if (!state.lastTs) state.lastTs = ts;
  const dt = Math.min(0.033, (ts - state.lastTs) / 1000);
  state.lastTs = ts;

  const p = getParams();
  if (state.running) {
    state.simTime += dt;
    state.unwAng += p.unwOmega * dt;
    state.webPhase += p.unwLinear * dt * 0.16;
  }

  const kin = feederKinematics(state.simTime, p);
  const unwPos = p.unwLinear * state.simTime;
  const cycleIdx = Math.floor(state.simTime / p.cycleT);
  const feederTotalPos = cycleIdx * p.feedLength + kin.pos;
  const bufferMM = (unwPos - feederTotalPos) / 2;
  const bufferPX = bufferMM * 0.35;

  const tc = ((state.simTime % p.cycleT) + p.cycleT) % p.cycleT;
  const stopPhase = tc > p.moveT ? (tc - p.moveT) / p.stopT : -1;
  const cutterLift = stopPhase >= 0 ? Math.sin(Math.PI * Math.min(1, stopPhase)) ** 2 : 0;

  const vUnw = state.running ? p.unwLinear : 0;
  const vFeed = state.running ? kin.vel : 0;
  const aFeed = state.running ? kin.acc : 0;
  const bufferVel = state.running ? (bufferMM - state.bufferPosPrev) / Math.max(1e-6, dt) : 0;
  const bufferAcc = state.running ? (bufferVel - state.feederVelPrev) / Math.max(1e-6, dt) : 0;

  state.bufferPosPrev = bufferMM;
  state.feederVelPrev = bufferVel;

  pushHistory({
    t: state.simTime,
    'Unw Vel': vUnw,
    'Feeder Vel': vFeed,
    'Buffer Vel': bufferVel,
    'Unw Acc': 0,
    'Feeder Acc': aFeed,
    'Buffer Acc': bufferAcc,
    'Feeder Pos': feederTotalPos,
    'Buffer Pos': bufferMM,
  });

  drawMachine(p, kin, bufferPX, cutterLift, state.webPhase * 0.06);
  drawChart(velChart, ['Unw Vel', 'Feeder Vel', 'Buffer Vel'], ['#67f8ac', '#ff7ab3', '#5bc6ff'], 'Velocity', 'mm/s');
  drawChart(accChart, ['Unw Acc', 'Feeder Acc', 'Buffer Acc'], ['#67f8ac', '#ff7ab3', '#5bc6ff'], 'Acceleration', 'mm/s²');
  drawChart(posChart, ['Feeder Pos', 'Buffer Pos'], ['#ff7ab3', '#5bc6ff'], 'Position', 'mm');
  updateDerivedText(p, kin, bufferMM, cutterLift);

  requestAnimationFrame(tick);
}

for (const el of Object.values(inputs)) el.addEventListener('input', updateValueLabels);

document.getElementById('startBtn').addEventListener('click', () => {
  state.running = !state.running;
  document.getElementById('startBtn').textContent = state.running ? 'Pause' : 'Start';
});

document.getElementById('resetBtn').addEventListener('click', () => {
  resetState();
  document.getElementById('startBtn').textContent = 'Start';
});

updateValueLabels();
resetState();
requestAnimationFrame(tick);

const ids = [
  'unwinderDeg', 'feedLength', 'accTime', 'decTime', 'feederVmax',
  'dR1', 'dR2', 'dR3', 'dUnw', 'dFeed', 'dBuf'
];

const state = {
  running: false,
  simTime: 0,
  lastTs: 0,
  unwAng: 0,
  feederAng: 0,
  webPhase: 0,
  bufferPosPrev: 0,
  bufferVelPrev: 0,
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
  for (const [id, el] of Object.entries(inputs)) {
    document.getElementById(`${id}Val`).textContent = el.value;
  }
}

function getParams() {
  const p = Object.fromEntries(Object.entries(inputs).map(([k, el]) => [k, Number(el.value)]));
  p.unwOmega = (p.unwinderDeg * Math.PI) / 180;
  p.rUnw = p.dUnw / 2;
  p.rFeed = p.dFeed / 2;
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

  if (tv >= 0) {
    const denom = tv + 0.5 * (ta + td);
    vPeak = Math.min(p.feederVmax, p.feedLength / Math.max(1e-8, denom));
    if (vPeak * denom < p.feedLength) vPeak = p.feedLength / denom;
  } else {
    const sum = ta + td;
    ta = p.moveT * (ta / sum);
    td = p.moveT - ta;
    tv = 0;
    vPeak = (2 * p.feedLength) / Math.max(1e-8, p.moveT);
  }

  const aUp = vPeak / Math.max(1e-8, ta);
  const aDn = vPeak / Math.max(1e-8, td);

  if (tc < ta) return { pos: 0.5 * aUp * tc * tc, vel: aUp * tc, acc: aUp, moving: true };
  if (tc < ta + tv) {
    const t2 = tc - ta;
    return { pos: 0.5 * aUp * ta * ta + vPeak * t2, vel: vPeak, acc: 0, moving: true };
  }

  const t3 = tc - ta - tv;
  const p1 = 0.5 * aUp * ta * ta + vPeak * tv;
  return {
    pos: p1 + vPeak * t3 - 0.5 * aDn * t3 * t3,
    vel: Math.max(0, vPeak - aDn * t3),
    acc: -aDn,
    moving: true,
  };
}

function pushHistory(sample) {
  state.history.push(sample);
  while (state.history.length && state.history[0].t < state.simTime - 10) state.history.shift();
}

function drawRoller(x, y, r, label, angle = 0) {
  const ctx = mctx;
  const grad = ctx.createRadialGradient(x - r * 0.3, y - r * 0.35, r * 0.12, x, y, r);
  grad.addColorStop(0, '#9ce8ff');
  grad.addColorStop(0.58, '#2da5ea');
  grad.addColorStop(1, '#0d346f');

  ctx.shadowBlur = 24;
  ctx.shadowColor = 'rgba(75,207,255,0.9)';
  ctx.fillStyle = grad;
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fill();

  ctx.shadowBlur = 0;
  ctx.lineWidth = 2;
  ctx.strokeStyle = '#8ef3ff';
  ctx.beginPath();
  ctx.arc(x, y, r * 0.88, 0, Math.PI * 2);
  ctx.stroke();

  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(angle);
  ctx.strokeStyle = 'rgba(188,248,255,0.9)';
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

function pointOnCircle(c, a, scale = 0.95) {
  return { x: c.x + c.r * scale * Math.cos(a), y: c.y + c.r * scale * Math.sin(a) };
}

function buildWebPath(ctx, S) {
  // Orthogonal, externally-tangent web route (matching requested schematic style).
  const unwL = pointOnCircle(S.unw, Math.PI);
  const unwT = pointOnCircle(S.unw, -Math.PI / 2);

  const r1L = pointOnCircle(S.r1, Math.PI);
  const r1T = pointOnCircle(S.r1, -Math.PI / 2);

  const r2T = pointOnCircle(S.r2, -Math.PI / 2);
  const r2R = pointOnCircle(S.r2, 0);

  const bufL = pointOnCircle(S.buf, Math.PI);
  const bufR = pointOnCircle(S.buf, 0);

  const r3L = pointOnCircle(S.r3, Math.PI);
  const r3T = pointOnCircle(S.r3, -Math.PI / 2);

  const feedT = pointOnCircle(S.feed, -Math.PI / 2);
  const feedR = pointOnCircle(S.feed, 0);

  ctx.beginPath();

  // Unwinder left side up + wrap to top.
  ctx.moveTo(unwL.x, unwL.y + S.unw.r * 0.95);
  ctx.lineTo(unwL.x, unwL.y);
  ctx.arc(S.unw.x, S.unw.y, S.unw.r * 0.95, Math.PI, -Math.PI / 2, true);

  // Link to Roller1 left and wrap to top.
  ctx.lineTo(r1L.x, r1L.y);
  ctx.arc(S.r1.x, S.r1.y, S.r1.r * 0.95, Math.PI, -Math.PI / 2, true);

  // Top straight to Roller2 top, then down right side to buffer left side.
  ctx.lineTo(r2T.x, r2T.y);
  ctx.arc(S.r2.x, S.r2.y, S.r2.r * 0.95, -Math.PI / 2, 0, false);
  ctx.lineTo(r2R.x, bufL.y);

  // Active buffer wrap (left to right across lower side).
  ctx.lineTo(bufL.x, bufL.y);
  ctx.arc(S.buf.x, S.buf.y, S.buf.r * 0.95, Math.PI, 0, true);

  // Up to Roller3 left, wrap to top.
  ctx.lineTo(bufR.x, r3L.y);
  ctx.lineTo(r3L.x, r3L.y);
  ctx.arc(S.r3.x, S.r3.y, S.r3.r * 0.95, Math.PI, -Math.PI / 2, true);

  // Top straight to feeder top, then to right side and small tail.
  ctx.lineTo(feedT.x, feedT.y);
  ctx.arc(S.feed.x, S.feed.y, S.feed.r * 0.95, -Math.PI / 2, 0, false);
  ctx.lineTo(feedR.x + S.feed.r * 0.72, feedR.y);
}


function drawMachine(p, kin, bufferY, cutterLift) {
  const ctx = mctx;
  const w = machineCanvas.width;
  const h = machineCanvas.height;
  ctx.clearRect(0, 0, w, h);

  const S = {
    r1: { x: 130, y: 90, r: p.dR1 * 0.35 },
    r2: { x: 450, y: 90, r: p.dR2 * 0.35 },
    r3: { x: 780, y: 90, r: p.dR3 * 0.35 },
    unw: { x: 130, y: 300, r: p.dUnw * 0.35 },
    feed: { x: 1030, y: 90, r: p.dFeed * 0.35 },
    buf: { x: 620, y: 300 + bufferY, r: p.dBuf * 0.35 },
  };

  buildWebPath(ctx, S);
  ctx.lineWidth = 12;
  ctx.strokeStyle = 'rgba(255,45,130,0.12)';
  ctx.shadowColor = 'rgba(255,70,160,0.95)';
  ctx.shadowBlur = 25;
  ctx.stroke();

  buildWebPath(ctx, S);
  ctx.lineWidth = 4.5;
  ctx.strokeStyle = '#ff4d7f';
  ctx.shadowBlur = 16;
  ctx.stroke();

  if (state.running) {
    buildWebPath(ctx, S);
    ctx.lineWidth = 2.4;
    ctx.strokeStyle = '#ffd6e4';
    ctx.setLineDash([11, 20]);
    ctx.lineDashOffset = -state.webPhase;
    ctx.shadowBlur = 10;
    ctx.stroke();
    ctx.setLineDash([]);
  }
  ctx.shadowBlur = 0;

  drawRoller(S.r1.x, S.r1.y, S.r1.r, 'Roller1', state.unwAng * 0.5);
  drawRoller(S.r2.x, S.r2.y, S.r2.r, 'Roller2', state.unwAng * 0.42);
  drawRoller(S.r3.x, S.r3.y, S.r3.r, 'Roller3', state.feederAng * 0.8);
  drawRoller(S.unw.x, S.unw.y, S.unw.r, 'Unwin\nder', state.unwAng);
  drawRoller(S.feed.x, S.feed.y, S.feed.r, 'Feeder', state.feederAng);
  drawRoller(S.buf.x, S.buf.y, S.buf.r, 'Active\nBuffer', -state.unwAng * 0.3);

  const cutterX = S.feed.x + S.feed.r + 46;
  const topY = 30;
  const down = 78 * cutterLift;

  ctx.strokeStyle = 'rgba(130,250,255,0.7)';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(cutterX + 8, 6);
  ctx.lineTo(cutterX + 8, topY + down);
  ctx.stroke();

  ctx.shadowBlur = 20;
  ctx.shadowColor = 'rgba(122,255,245,0.85)';
  ctx.fillStyle = '#8efff5';
  ctx.fillRect(cutterX - 10, topY - 16, 38, 12);
  ctx.fillStyle = '#cafffa';
  ctx.fillRect(cutterX, topY + down, 16, 58);
  ctx.shadowBlur = 0;

  ctx.fillStyle = '#bcefff';
  ctx.font = '14px sans-serif';
  ctx.textAlign = 'left';
  ctx.fillText(`Cycle ${fmt(p.cycleT, ' s')} | Move ${fmt(p.moveT, ' s')} | Stop ${fmt(p.stopT, ' s')}`, 14, h - 40);
  ctx.fillText(`Feeder: ${kin.moving ? '이송중' : '정지'} | Buffer 보정: ${fmt(bufferY / 0.35, ' mm')} | Cutter: ${cutterLift > 0.5 ? 'DOWN' : 'UP'}`, 14, h - 18);
}

function drawAxes(ctx, x0, y0, w, h, tMin, tMax, yMin, yMax, unit) {
  ctx.strokeStyle = 'rgba(125,191,238,0.45)';
  ctx.strokeRect(x0, y0, w, h);

  ctx.fillStyle = '#a9dfff';
  ctx.font = '11px sans-serif';

  const yTicks = 5;
  for (let i = 0; i <= yTicks; i++) {
    const ratio = i / yTicks;
    const y = y0 + h * ratio;
    const value = yMax - (yMax - yMin) * ratio;
    ctx.strokeStyle = 'rgba(92,153,214,0.2)';
    ctx.beginPath();
    ctx.moveTo(x0, y);
    ctx.lineTo(x0 + w, y);
    ctx.stroke();
    ctx.fillText(value.toFixed(1), x0 - 38, y + 4);
  }

  const xTicks = 5;
  for (let i = 0; i <= xTicks; i++) {
    const ratio = i / xTicks;
    const x = x0 + w * ratio;
    const value = tMin + (tMax - tMin) * ratio;
    ctx.strokeStyle = 'rgba(92,153,214,0.2)';
    ctx.beginPath();
    ctx.moveTo(x, y0);
    ctx.lineTo(x, y0 + h);
    ctx.stroke();
    ctx.fillText(`${value.toFixed(1)} s`, x - 12, y0 + h + 14);
  }

  ctx.fillText(unit, 6, y0 + 12);
}

function drawChart(ctx, keys, colors, title, unit) {
  const W = ctx.canvas.width;
  const H = ctx.canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#020711';
  ctx.fillRect(0, 0, W, H);

  const pad = { l: 50, r: 12, t: 22, b: 30 };
  const x0 = pad.l;
  const y0 = pad.t;
  const w = W - pad.l - pad.r;
  const h = H - pad.t - pad.b;

  if (state.history.length < 2) {
    ctx.fillStyle = '#a9dfff';
    ctx.fillText(`${title} (${unit})`, 12, 16);
    return;
  }

  const tMin = state.history[0].t;
  const tMax = state.history[state.history.length - 1].t;
  let yMin = Infinity;
  let yMax = -Infinity;

  for (const sample of state.history) {
    for (const key of keys) {
      yMin = Math.min(yMin, sample[key]);
      yMax = Math.max(yMax, sample[key]);
    }
  }

  if (Math.abs(yMax - yMin) < 1e-9) {
    yMin -= 1;
    yMax += 1;
  }

  const p = (yMax - yMin) * 0.1;
  yMin -= p;
  yMax += p;

  const sx = (t) => x0 + ((t - tMin) / Math.max(1e-6, tMax - tMin)) * w;
  const sy = (v) => y0 + h - ((v - yMin) / (yMax - yMin)) * h;

  drawAxes(ctx, x0, y0, w, h, tMin, tMax, yMin, yMax, unit);

  ctx.strokeStyle = 'rgba(173,226,255,0.35)';
  ctx.beginPath();
  ctx.moveTo(x0, sy(0));
  ctx.lineTo(x0 + w, sy(0));
  ctx.stroke();

  keys.forEach((key, i) => {
    ctx.shadowBlur = 12;
    ctx.shadowColor = colors[i];
    ctx.strokeStyle = colors[i];
    ctx.lineWidth = 2;
    ctx.beginPath();
    state.history.forEach((sample, idx) => {
      const X = sx(sample.t);
      const Y = sy(sample[key]);
      if (idx === 0) ctx.moveTo(X, Y);
      else ctx.lineTo(X, Y);
    });
    ctx.stroke();
    ctx.shadowBlur = 0;
  });

  ctx.fillStyle = '#d5f3ff';
  ctx.font = '12px sans-serif';
  ctx.fillText(`${title} (${unit})`, x0, 14);

  keys.forEach((key, i) => {
    ctx.fillStyle = colors[i];
    ctx.fillRect(W - 188, 8 + i * 14, 10, 10);
    ctx.fillStyle = '#dbf5ff';
    ctx.fillText(key, W - 174, 17 + i * 14);
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
  state.feederAng = 0;
  state.webPhase = 0;
  state.bufferPosPrev = 0;
  state.bufferVelPrev = 0;
  state.history = [];
}

function tick(ts) {
  if (!state.lastTs) state.lastTs = ts;
  const dt = Math.min(0.033, (ts - state.lastTs) / 1000);
  state.lastTs = ts;

  const p = getParams();
  const kin = feederKinematics(state.simTime, p);

  if (state.running) {
    state.simTime += dt;
    state.unwAng += p.unwOmega * dt;
    state.feederAng += (kin.vel / Math.max(1e-6, p.rFeed)) * dt;
    state.webPhase += p.unwLinear * dt * 0.12;
  }

  const kinNow = feederKinematics(state.simTime, p);
  const unwPos = p.unwLinear * state.simTime;
  const cycleIdx = Math.floor(state.simTime / p.cycleT);
  const feederTotalPos = cycleIdx * p.feedLength + kinNow.pos;
  const bufferMM = (unwPos - feederTotalPos) / 2;
  const bufferPX = bufferMM * 0.35;

  const tc = ((state.simTime % p.cycleT) + p.cycleT) % p.cycleT;
  const stopPhase = tc > p.moveT ? (tc - p.moveT) / p.stopT : -1;
  const cutterLift = stopPhase >= 0 ? Math.sin(Math.PI * Math.min(1, stopPhase)) ** 2 : 0;

  const vUnw = state.running ? p.unwLinear : 0;
  const vFeed = state.running ? kinNow.vel : 0;
  const aFeed = state.running ? kinNow.acc : 0;

  const bufferVel = state.running ? (bufferMM - state.bufferPosPrev) / Math.max(1e-6, dt) : 0;
  const bufferAcc = state.running ? (bufferVel - state.bufferVelPrev) / Math.max(1e-6, dt) : 0;

  state.bufferPosPrev = bufferMM;
  state.bufferVelPrev = bufferVel;

  pushHistory({
    t: state.simTime,
    'Unw Vel': vUnw,
    'Feeder Vel': vFeed,
    'Buffer Vel': bufferVel,
    'Unw Acc': 0,
    'Feeder Acc': aFeed,
    'Buffer Acc': bufferAcc,
    'Buffer Pos': bufferMM,
  });

  drawMachine(p, kinNow, bufferPX, cutterLift);
  drawChart(velChart, ['Unw Vel', 'Feeder Vel', 'Buffer Vel'], ['#67f8ac', '#ff7ab3', '#5bc6ff'], 'Velocity', 'mm/s');
  drawChart(accChart, ['Unw Acc', 'Feeder Acc', 'Buffer Acc'], ['#67f8ac', '#ff7ab3', '#5bc6ff'], 'Acceleration', 'mm/s²');
  drawChart(posChart, ['Buffer Pos'], ['#5bc6ff'], 'Position', 'mm');
  updateDerivedText(p, kinNow, bufferMM, cutterLift);

  requestAnimationFrame(tick);
}

for (const el of Object.values(inputs)) {
  el.addEventListener('input', updateValueLabels);
}

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

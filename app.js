const rangeIds = [
  'unwinderDeg', 'feedLength', 'accTime', 'decTime', 'feederVmax',
  'dR1', 'dR2', 'dR3', 'dUnw', 'dFeed', 'dBuf'
];

const directionIds = ['entryR1', 'entryR2', 'entryBuf', 'entryR3', 'entryFeed'];

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

const rangeInputs = Object.fromEntries(rangeIds.map((id) => [id, document.getElementById(id)]));
const directionInputs = Object.fromEntries(directionIds.map((id) => [id, document.getElementById(id)]));
const derivedEl = document.getElementById('derived');

const machineCanvas = document.getElementById('machineCanvas');
const mctx = machineCanvas.getContext('2d');
const velChart = document.getElementById('velChart').getContext('2d');
const accChart = document.getElementById('accChart').getContext('2d');
const posChart = document.getElementById('posChart').getContext('2d');

const fmt = (v, unit = '') => `${Number(v).toFixed(2)}${unit}`;

function updateValueLabels() {
  for (const [id, el] of Object.entries(rangeInputs)) {
    document.getElementById(`${id}Val`).textContent = el.value;
  }
}

function getParams() {
  const p = Object.fromEntries(Object.entries(rangeInputs).map(([k, el]) => [k, Number(el.value)]));
  for (const [k, el] of Object.entries(directionInputs)) p[k] = el.value;
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

function computeExternalTangent(c1, c2, side = 1) {
  const dx = c2.x - c1.x;
  const dy = c2.y - c1.y;
  const d = Math.hypot(dx, dy) || 1e-6;
  const ux = dx / d;
  const uy = dy / d;
  const nx = side * (-uy);
  const ny = side * ux;

  return {
    p1: { x: c1.x + nx * c1.r * 0.95, y: c1.y + ny * c1.r * 0.95 },
    p2: { x: c2.x + nx * c2.r * 0.95, y: c2.y + ny * c2.r * 0.95 },
  };
}

function angleOn(c, p) {
  return Math.atan2(p.y - c.y, p.x - c.x);
}

function segmentPointDistance(a, b, p) {
  const vx = b.x - a.x;
  const vy = b.y - a.y;
  const wx = p.x - a.x;
  const wy = p.y - a.y;
  const c1 = vx * wx + vy * wy;
  if (c1 <= 0) return Math.hypot(p.x - a.x, p.y - a.y);
  const c2 = vx * vx + vy * vy;
  if (c2 <= c1) return Math.hypot(p.x - b.x, p.y - b.y);
  const t = c1 / c2;
  const qx = a.x + t * vx;
  const qy = a.y + t * vy;
  return Math.hypot(p.x - qx, p.y - qy);
}

function chooseTangentSides(rollers) {
  const pairCount = rollers.length - 1;
  let best = null;

  for (let mask = 0; mask < (1 << pairCount); mask++) {
    const sides = Array.from({ length: pairCount }, (_, i) => ((mask >> i) & 1 ? 1 : -1));
    const tangents = sides.map((side, i) => computeExternalTangent(rollers[i], rollers[i + 1], side));

    const inPts = Array(rollers.length).fill(null);
    const outPts = Array(rollers.length).fill(null);
    for (let i = 0; i < tangents.length; i++) {
      outPts[i] = tangents[i].p1;
      inPts[i + 1] = tangents[i].p2;
    }

    let score = 0;

    // User-required contact quadrants.
    const r2 = rollers[2], b = rollers[3], f = rollers[5];
    if (!(outPts[2].x > r2.x && outPts[2].y > r2.y)) score += 5000; // R2 exit lower-right

    // Active Buffer direction constraint: entry from left, exit to right (upper-right).
    if (!(inPts[3].x < b.x)) score += 7000;
    if (!(outPts[3].x > b.x && outPts[3].y < b.y)) score += 7000;

    if (!(inPts[5].x < f.x && inPts[5].y > f.y)) score += 5000;     // Feeder entry lower-left

    // Penalize any straight tangent segment that cuts too close to non-adjacent rollers.
    for (let i = 0; i < rollers.length - 1; i++) {
      const a = outPts[i];
      const c = inPts[i + 1];
      for (let j = 0; j < rollers.length; j++) {
        if (j === i || j === i + 1) continue;
        const d = segmentPointDistance(a, c, rollers[j]);
        const limit = rollers[j].r * 0.93;
        if (d < limit) score += (limit - d) * 400;
      }
    }

    if (!best || score < best.score) best = { sides, score };
  }

  return best ? best.sides : [-1, -1, 1, -1, -1];
}

function tangentPointsFromPoint(point, circle) {
  const dx = point.x - circle.x;
  const dy = point.y - circle.y;
  const r = circle.r * 0.95;
  const d2 = dx * dx + dy * dy;
  const d = Math.sqrt(Math.max(d2, 1e-9));
  if (d <= r) return [];

  const alpha = Math.atan2(dy, dx);
  const beta = Math.acos(r / d);
  const a1 = alpha + beta;
  const a2 = alpha - beta;
  return [
    { x: circle.x + r * Math.cos(a1), y: circle.y + r * Math.sin(a1), a: a1 },
    { x: circle.x + r * Math.cos(a2), y: circle.y + r * Math.sin(a2), a: a2 },
  ];
}

function cwDelta(a0, a1) {
  return ((a1 - a0) % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2);
}

function ccwDelta(a0, a1) {
  return ((a0 - a1) % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2);
}

const directionAngles = {
  left: Math.PI,
  top: -Math.PI / 2,
  right: 0,
  bottom: Math.PI / 2,
  leftTop: -3 * Math.PI / 4,
  rightTop: -Math.PI / 4,
  rightBottom: Math.PI / 4,
  leftBottom: 3 * Math.PI / 4,
};

function circularDiff(a, b) {
  const d = ((a - b + Math.PI) % (Math.PI * 2)) - Math.PI;
  return Math.abs(d);
}

function entryPenalty(point, circle, direction) {
  if (!direction || direction === 'auto') return 0;
  const target = directionAngles[direction];
  if (target === undefined) return 0;
  const actual = Math.atan2(point.y - circle.y, point.x - circle.x);
  return circularDiff(actual, target) * 120;
}

function buildWebRoute(S, p) {
  const unw = S.unw;
  const r1 = S.r1;
  const r2 = S.r2;
  const buf = S.buf;
  const r3 = S.r3;
  const feed = S.feed;

  // 1) Start at unwinder 9 o'clock.
  const start = pointOnCircle(unw, Math.PI);

  // 2) Entry line from unwinder start to Roller1 nearest tangent point.
  const r1TangentsFromStart = tangentPointsFromPoint(start, r1);
  const r1In = r1TangentsFromStart
    .map((pt) => ({ pt, score: Math.hypot(pt.x - start.x, pt.y - start.y) + entryPenalty(pt, r1, p.entryR1) }))
    .sort((a, b) => a.score - b.score)[0].pt;

  // 3) Roller1 clockwise arc -> tangent to Roller2.
  const t12m = computeExternalTangent(r1, r2, -1);
  const t12p = computeExternalTangent(r1, r2, 1);
  const aR1In = angleOn(r1, r1In);
  const candR1 = [
    { side: -1, r1Out: t12m.p1, r2In: t12m.p2 },
    { side: 1, r1Out: t12p.p1, r2In: t12p.p2 },
  ].map((c) => ({
    ...c,
    arcScore: cwDelta(aR1In, angleOn(r1, c.r1Out)) + entryPenalty(c.r2In, r2, p.entryR2),
  }))
   .sort((a, b) => a.arcScore - b.arcScore)[0];

  // 4) Roller2 clockwise arc -> ActiveBuffer left-side nearest tangent.
  const t2Bm = computeExternalTangent(r2, buf, -1);
  const t2Bp = computeExternalTangent(r2, buf, 1);
  const aR2In = angleOn(r2, candR1.r2In);
  const candR2 = [
    { r2Out: t2Bm.p1, bIn: t2Bm.p2 },
    { r2Out: t2Bp.p1, bIn: t2Bp.p2 },
  ].map((c) => {
    const leftPenalty = c.bIn.x < buf.x ? 0 : 10000;
    const arcScore = cwDelta(aR2In, angleOn(r2, c.r2Out));
    return { ...c, score: leftPenalty + arcScore + entryPenalty(c.bIn, buf, p.entryBuf) };
  }).sort((a, b) => a.score - b.score)[0];

  // 5) ActiveBuffer counter-clockwise arc -> right-side tangent to Roller3 left side.
  const tB3m = computeExternalTangent(buf, r3, -1);
  const tB3p = computeExternalTangent(buf, r3, 1);
  const aBIn = angleOn(buf, candR2.bIn);
  const candB = [
    { bOut: tB3m.p1, r3In: tB3m.p2 },
    { bOut: tB3p.p1, r3In: tB3p.p2 },
  ].map((c) => {
    const rightPenalty = c.bOut.x > buf.x ? 0 : 10000;
    const leftR3Penalty = c.r3In.x < r3.x ? 0 : 10000;
    const arcScore = ccwDelta(aBIn, angleOn(buf, c.bOut));
    return { ...c, score: rightPenalty + leftR3Penalty + arcScore + entryPenalty(c.r3In, r3, p.entryR3) };
  }).sort((a, b) => a.score - b.score)[0];

  // 6) Roller3 clockwise arc -> Feeder left-bottom shortest tangent line.
  const t3Fm = computeExternalTangent(r3, feed, -1);
  const t3Fp = computeExternalTangent(r3, feed, 1);
  const aR3In = angleOn(r3, candB.r3In);
  const candR3 = [
    { r3Out: t3Fm.p1, fIn: t3Fm.p2 },
    { r3Out: t3Fp.p1, fIn: t3Fp.p2 },
  ].map((c) => {
    const feederQuadPenalty = c.fIn.x < feed.x && c.fIn.y > feed.y ? 0 : 10000;
    const arcScore = cwDelta(aR3In, angleOn(r3, c.r3Out));
    const lineScore = Math.hypot(c.fIn.x - c.r3Out.x, c.fIn.y - c.r3Out.y);
    return { ...c, score: feederQuadPenalty + arcScore + lineScore * 0.01 + entryPenalty(c.fIn, feed, p.entryFeed) };
  }).sort((a, b) => a.score - b.score)[0];

  return {
    start,
    r1In,
    r1Out: candR1.r1Out,
    r2In: candR1.r2In,
    r2Out: candR2.r2Out,
    bIn: candR2.bIn,
    bOut: candB.bOut,
    r3In: candB.r3In,
    r3Out: candR3.r3Out,
    fIn: candR3.fIn,
    rollers: { unw, r1, r2, buf, r3, feed },
  };
}

function buildWebPath(ctx, S, p) {
  const R = buildWebRoute(S, p);
  const { unw, r1, r2, buf, r3 } = R.rollers;

  ctx.beginPath();
  ctx.moveTo(R.start.x, R.start.y);
  ctx.lineTo(R.r1In.x, R.r1In.y);

  // Roller1 clockwise
  ctx.arc(r1.x, r1.y, r1.r * 0.95, angleOn(r1, R.r1In), angleOn(r1, R.r1Out), false);
  ctx.lineTo(R.r2In.x, R.r2In.y);

  // Roller2 clockwise
  ctx.arc(r2.x, r2.y, r2.r * 0.95, angleOn(r2, R.r2In), angleOn(r2, R.r2Out), false);
  ctx.lineTo(R.bIn.x, R.bIn.y);

  // ActiveBuffer counter-clockwise
  ctx.arc(buf.x, buf.y, buf.r * 0.95, angleOn(buf, R.bIn), angleOn(buf, R.bOut), true);
  ctx.lineTo(R.r3In.x, R.r3In.y);

  // Roller3 clockwise then straight to Feeder left-bottom entry
  ctx.arc(r3.x, r3.y, r3.r * 0.95, angleOn(r3, R.r3In), angleOn(r3, R.r3Out), false);
  ctx.lineTo(R.fIn.x, R.fIn.y);

  return R;
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

  buildWebPath(ctx, S, p);
  ctx.lineWidth = 12;
  ctx.strokeStyle = 'rgba(255,45,130,0.12)';
  ctx.shadowColor = 'rgba(255,70,160,0.95)';
  ctx.shadowBlur = 25;
  ctx.stroke();

  buildWebPath(ctx, S, p);
  ctx.lineWidth = 4.5;
  ctx.strokeStyle = '#ff4d7f';
  ctx.shadowBlur = 16;
  ctx.stroke();

  if (state.running) {
    buildWebPath(ctx, S, p);
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

for (const el of Object.values(rangeInputs)) el.addEventListener('input', updateValueLabels);
for (const el of Object.values(directionInputs)) el.addEventListener('change', updateValueLabels);

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


//  PID CONTROLLER SIMULATION


const PID = (() => {
  const canvas = document.getElementById('pidCanvas');
  const ctx = canvas.getContext('2d');

  let raf, running = false;
  let t = 0, dt = 0.016;
  let pidX, pidY, pidVx, pidVy;
  let noX, noY, noVx, noVy;
  let errX = 0, errY = 0, prevErrX = 0, prevErrY = 0;
  let intX = 0, intY = 0;
  let pidTrail = [], noTrail = [], refTrail = [];
  let errHistory = [];
  let stepCount = 0, rmsAcc = 0, maxErr = 0;
  let pathMode = 'figure8';
  let errChart = null;

  //  Parameter helpers 
  const gp = id => parseFloat(document.getElementById(id).value);

  // Reference path 
  function refPath(tt) {
    const a = 110, b = 70, r = 100;
    switch (pathMode) {
      case 'circle':
        return { x: r * Math.cos(tt), y: r * Math.sin(tt) };
      case 'zigzag': {
        const pts = [
          { x: -120, y: 0 }, { x: -70, y: 70 }, { x: 0, y: -70 },
          { x: 70, y: 70 }, { x: 120, y: 0 }, { x: 70, y: -70 },
          { x: 0, y: 70 }, { x: -70, y: -70 }, { x: -120, y: 0 }
        ];
        const cycle = 2 * Math.PI;
        const seg = Math.floor(((tt % cycle + cycle) % cycle) / (cycle / 8));
        const frac = (((tt % cycle + cycle) % cycle) % (cycle / 8)) / (cycle / 8);
        const s = Math.min(seg, 7);
        return {
          x: pts[s].x + (pts[s + 1].x - pts[s].x) * frac,
          y: pts[s].y + (pts[s + 1].y - pts[s].y) * frac
        };
      }
      case 'spiral': {
        const rad = 30 + 70 * Math.abs(Math.sin(tt * 0.25));
        return { x: rad * Math.cos(tt * 1.5), y: rad * Math.sin(tt * 1.5) };
      }
      default: // figure8
        return { x: a * Math.sin(tt), y: b * Math.sin(2 * tt) };
    }
  }

  //  Reset state 
  function reset() {
    t = 0; stepCount = 0; rmsAcc = 0; maxErr = 0;
    errX = 0; errY = 0; prevErrX = 0; prevErrY = 0; intX = 0; intY = 0;
    const p0 = refPath(0);
    pidX = p0.x; pidY = p0.y; pidVx = 0; pidVy = 0;
    noX = p0.x; noY = p0.y; noVx = 0; noVy = 0;
    pidTrail = []; noTrail = []; refTrail = []; errHistory = [];
    updateMetrics();
    draw();
  }

  //  Simulation step 
  function step() {
    const kp = gp('kp'), ki = gp('ki'), kd = gp('kd');
    const spd = gp('speed'), noise = gp('noise'), mass = gp('mass');
    const n = () => (Math.random() - 0.5) * noise;

    t += dt * spd;
    const ref = refPath(t);

    // PID
    prevErrX = errX; prevErrY = errY;
    errX = ref.x - pidX + n();
    errY = ref.y - pidY + n();
    intX = Math.max(-60, Math.min(60, intX + errX * dt));
    intY = Math.max(-60, Math.min(60, intY + errY * dt));
    const dErrX = (errX - prevErrX) / dt;
    const dErrY = (errY - prevErrY) / dt;
    const fx = kp * errX + ki * intX + kd * dErrX;
    const fy = kp * errY + ki * intY + kd * dErrY;
    pidVx += (fx / mass) * dt;
    pidVy += (fy / mass) * dt;
    pidVx *= 0.84; pidVy *= 0.84;
    pidX += pidVx * dt;
    pidY += pidVy * dt;

    // No control robot
    noVx += ((ref.x - noX) * 0.06 / mass + n() * 0.35) * dt;
    noVy += ((ref.y - noY) * 0.06 / mass + n() * 0.35) * dt;
    noVx *= 0.9; noVy *= 0.9;
    noX += noVx * dt;
    noY += noVy * dt;

    const e = Math.sqrt(errX ** 2 + errY ** 2);
    stepCount++;
    rmsAcc += e * e;
    if (e > maxErr) maxErr = e;
    errHistory.push(e);
    if (errHistory.length > 300) errHistory.shift();

    const MAX = 400;
    pidTrail.push({ x: pidX, y: pidY }); if (pidTrail.length > MAX) pidTrail.shift();
    noTrail.push({ x: noX, y: noY }); if (noTrail.length > MAX) noTrail.shift();
    refTrail.push({ x: ref.x, y: ref.y }); if (refTrail.length > MAX) refTrail.shift();

    updateMetrics();
  }

  //  Canvas helpers 
  function toC(x, y) {
    const cw = canvas.width, ch = canvas.height;
    return { cx: cw / 2 + x, cy: ch / 2 - y };
  }

  function drawGrid() {
    ctx.strokeStyle = 'rgba(0,255,136,0.04)';
    ctx.lineWidth = 1;
    const step = 40;
    for (let x = canvas.width % step; x < canvas.width; x += step) {
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();
    }
    for (let y = canvas.height % step; y < canvas.height; y += step) {
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
    }
    // Axes
    ctx.strokeStyle = 'rgba(0,255,136,0.08)';
    ctx.lineWidth = 1;
    const cx = canvas.width / 2, cy = canvas.height / 2;
    ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, canvas.height); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(canvas.width, cy); ctx.stroke();
  }

  function drawRefPath() {
    const steps = 300;
    ctx.beginPath();
    for (let i = 0; i <= steps; i++) {
      const tt = (i / steps) * Math.PI * 2;
      const p = refPath(tt);
      const { cx, cy } = toC(p.x, p.y);
      i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy);
    }
    ctx.strokeStyle = '#4488ff';
    ctx.lineWidth = 1;
    ctx.globalAlpha = 0.3;
    ctx.setLineDash([5, 5]);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  }

  function drawTrail(trail, color, alpha, width) {
    if (trail.length < 2) return;
    ctx.beginPath();
    const p0 = toC(trail[0].x, trail[0].y);
    ctx.moveTo(p0.cx, p0.cy);
    for (let i = 1; i < trail.length; i++) {
      const p = toC(trail[i].x, trail[i].y);
      ctx.lineTo(p.cx, p.cy);
    }
    ctx.strokeStyle = color;
    ctx.globalAlpha = alpha;
    ctx.lineWidth = width;
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  function drawRobot(x, y, color, size, label) {
    const { cx, cy } = toC(x, y);
    ctx.beginPath();
    ctx.arc(cx, cy, size, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.shadowColor = color;
    ctx.shadowBlur = 12;
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.strokeStyle = '#0a0c0f';
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  function draw() {
    const cw = canvas.offsetWidth, ch = 380;
    canvas.width = cw; canvas.height = ch;
    ctx.clearRect(0, 0, cw, ch);
    ctx.fillStyle = '#0f1215';
    ctx.fillRect(0, 0, cw, ch);
    drawGrid();
    drawRefPath();
    drawTrail(noTrail, '#ff4444', 0.5, 1.5);
    drawTrail(pidTrail, '#00ff88', 0.85, 2);
    drawRobot(noX, noY, '#ff4444', 5, 'NC');
    drawRobot(pidX, pidY, '#00ff88', 8, 'PID');
    document.getElementById('pidTime').textContent = `t = ${t.toFixed(2)}s`;
  }

  // Error chart 
  function initErrChart() {
    if (errChart) errChart.destroy();
    errChart = new Chart(document.getElementById('pidErrChart'), {
      type: 'line',
      data: {
        labels: Array(300).fill(''),
        datasets: [{
          data: Array(300).fill(0),
          borderColor: '#00ff88',
          backgroundColor: 'rgba(0,255,136,0.07)',
          borderWidth: 1.5,
          pointRadius: 0,
          fill: true,
          tension: 0.3
        }]
      },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: { legend: { display: false } },
        scales: {
          x: { display: false },
          y: {
            min: 0,
            grid: { color: 'rgba(0,255,136,0.06)' },
            ticks: { color: '#4a5a4a', font: { size: 10, family: 'Space Mono' }, maxTicksLimit: 4 }
          }
        }
      }
    });
  }

  function updateErrChart() {
    if (!errChart) return;
    const padded = [...Array(300 - errHistory.length).fill(null), ...errHistory];
    errChart.data.datasets[0].data = padded;
    errChart.update('none');
  }

  function updateMetrics() {
    const rms = stepCount > 0 ? Math.sqrt(rmsAcc / stepCount) : 0;
    document.getElementById('mRms').textContent = rms.toFixed(1);
    document.getElementById('mMax').textContent = maxErr.toFixed(1);
    document.getElementById('mWind').textContent = Math.sqrt(intX ** 2 + intY ** 2).toFixed(1);
    document.getElementById('mSteps').textContent = stepCount;
    updateErrChart();
  }

  //  Animation loop 
  function loop() {
    step(); draw();
    raf = requestAnimationFrame(loop);
  }

  //  Bind sliders 
  function bindSlider(id, outId, dec) {
    const el = document.getElementById(id);
    const out = document.getElementById(outId);
    el.addEventListener('input', () => { out.textContent = parseFloat(el.value).toFixed(dec); });
  }

  //  Public init 
  function init() {
    bindSlider('kp', 'kpVal', 1);
    bindSlider('ki', 'kiVal', 2);
    bindSlider('kd', 'kdVal', 1);
    bindSlider('speed', 'speedVal', 1);
    bindSlider('noise', 'noiseVal', 1);
    bindSlider('mass', 'massVal', 1);

    document.querySelectorAll('.path-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.path-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        pathMode = btn.dataset.path;
        stopSim(); reset();
      });
    });

    document.getElementById('pidRunBtn').addEventListener('click', () => {
      if (running) { stopSim(); } else { startSim(); }
    });
    document.getElementById('pidResetBtn').addEventListener('click', () => { stopSim(); reset(); });

    initErrChart();
    reset();
  }

  function startSim() {
    running = true;
    document.getElementById('pidRunBtn').textContent = '⏸ PAUSE';
    document.getElementById('statusPill').querySelector('.status-dot').classList.add('running');
    document.getElementById('statusText').textContent = 'RUNNING';
    loop();
  }

  function stopSim() {
    cancelAnimationFrame(raf);
    running = false;
    document.getElementById('pidRunBtn').textContent = '▶ RUN';
    document.getElementById('statusPill').querySelector('.status-dot').classList.remove('running');
    document.getElementById('statusText').textContent = 'IDLE';
  }

  window.addEventListener('resize', () => { if (!running) draw(); });

  return { init };
})();

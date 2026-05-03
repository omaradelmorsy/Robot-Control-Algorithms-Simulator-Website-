
//  STATE SPACE CONTROL SIMULATION

// System: mass spring damper driven to setpoint via pole placement
//   ẋ = Ax + Bu,   u = -Kx
//   A = [[0, 1], [-wn^2, -2*zeta*wn]]
//   K chosen to place closed loop poles

const StateSpace = (() => {
  const canvas = document.getElementById('ssCanvas');
  const ctx = canvas.getContext('2d');

  let raf, running = false;
  let t = 0, dt = 0.02;
  let x = -120, v = 0;            // state: position, velocity
  let xHistory = [], vHistory = [], tHistory = [];
  let MAX_HIST = 500;
  let phaseChart = null, stateChart = null;

  const gp = id => parseFloat(document.getElementById(id).value);

  // Closed loop matrices with feedback u = -K*[x; v]
  // Pole placement: desired poles at -zeta*wn ± wn*sqrt(1-zeta^2)*j
  // For full state feedback: K = [wn^2 - omega_d^2,  2*zeta*wn - 2*sigma]
  // We simplify: K = [wn^2 * gain,  2*zeta*wn * gain]
  function getK() {
    const zeta = gp('zeta'), wn = gp('wn'), gain = gp('gain');
    return [wn * wn * gain, 2 * zeta * wn * gain];
  }

  function reset() {
    t = 0;
    x = gp('x0');
    v = gp('v0');
    xHistory = []; vHistory = []; tHistory = [];
    updateMatrixDisplay();
    draw();
    updateCharts();
  }

  function step() {
    const zeta = gp('zeta'), wn = gp('wn');
    const K = getK();
    // Open loop: A = [[0,1],[-wn^2, -2*zeta*wn]]
    // Closed-loop with u = -K*[x;v]: u = -(K[0]*x + K[1]*v)
    // Target is 0 (regulation); translate: e = x - setpoint=0
    const u = -(K[0] * x + K[1] * v);
    const ax = -wn * wn * x - 2 * zeta * wn * v + wn * wn * u / Math.max(K[0], 0.01);
    // RK4 integration
    const ax1 = ax;
    const v1 = v + ax1 * dt / 2;
    const x1 = x + v * dt / 2;
    const ax2 = -(wn * wn) * x1 - 2 * zeta * wn * v1 + wn * wn * u / Math.max(K[0], 0.01);
    v += ax2 * dt;
    x += v * dt;
    t += dt;
    xHistory.push(x);
    vHistory.push(v);
    tHistory.push(t);
    if (xHistory.length > MAX_HIST) { xHistory.shift(); vHistory.shift(); tHistory.shift(); }
    document.getElementById('ssOverlay').textContent = `t = ${t.toFixed(2)}s   x = ${x.toFixed(2)}   ẋ = ${v.toFixed(2)}`;
  }

  // Drawing the 1D robot moving on a track
  function draw() {
    const cw = canvas.offsetWidth, ch = 280;
    canvas.width = cw; canvas.height = ch;
    ctx.clearRect(0, 0, cw, ch);
    ctx.fillStyle = '#0f1215';
    ctx.fillRect(0, 0, cw, ch);

    // Grid
    ctx.strokeStyle = 'rgba(0,255,136,0.04)';
    ctx.lineWidth = 1;
    for (let gx = 0; gx < cw; gx += 40) {
      ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, ch); ctx.stroke();
    }
    for (let gy = 0; gy < ch; gy += 40) {
      ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(cw, gy); ctx.stroke();
    }

    // Tracking
    const trackY = ch / 2;
    ctx.strokeStyle = 'rgba(0,255,136,0.18)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(20, trackY); ctx.lineTo(cw - 20, trackY);
    ctx.stroke();

    // Tick marks
    const center = cw / 2;
    for (let px = -200; px <= 200; px += 20) {
      const sx = center + px;
      if (sx < 20 || sx > cw - 20) continue;
      ctx.strokeStyle = 'rgba(0,255,136,0.1)';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(sx, trackY - 6); ctx.lineTo(sx, trackY + 6); ctx.stroke();
      if (px % 40 === 0) {
        ctx.fillStyle = '#4a5a4a';
        ctx.font = '9px Space Mono';
        ctx.textAlign = 'center';
        ctx.fillText(px.toString(), sx, trackY + 18);
      }
    }

    // Target (setpoint 0)
    const targetX = center;
    ctx.strokeStyle = '#4488ff';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(targetX, trackY - 40); ctx.lineTo(targetX, trackY + 40); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#4488ff';
    ctx.font = '10px Space Mono';
    ctx.textAlign = 'center';
    ctx.fillText('TARGET', targetX, trackY - 48);

    // Robot
    const robotX = Math.max(25, Math.min(cw - 25, center + x));
    const robotSize = 18;

    // Spring visualization
    if (robotX !== center) {
      drawSpring(ctx, center, trackY - 8, robotX, trackY - 8);
    }

    // Robot body
    ctx.fillStyle = '#00ff88';
    ctx.shadowColor = '#00ff88';
    ctx.shadowBlur = 18;
    ctx.fillRect(robotX - robotSize, trackY - robotSize, robotSize * 2, robotSize * 2);
    ctx.shadowBlur = 0;
    ctx.strokeStyle = '#0a0c0f';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(robotX - robotSize, trackY - robotSize, robotSize * 2, robotSize * 2);

    // Velocity arrow
    if (Math.abs(v) > 0.5) {
      const arrowLen = Math.min(Math.abs(v) * 15, 60);
      const dir = v > 0 ? 1 : -1;
      ctx.strokeStyle = '#ffcc00';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(robotX, trackY);
      ctx.lineTo(robotX + dir * arrowLen, trackY);
      ctx.stroke();
      // Arrowhead
      ctx.fillStyle = '#ffcc00';
      ctx.beginPath();
      ctx.moveTo(robotX + dir * arrowLen, trackY - 5);
      ctx.lineTo(robotX + dir * arrowLen + dir * 8, trackY);
      ctx.lineTo(robotX + dir * arrowLen, trackY + 5);
      ctx.fill();
    }

    // Labels
    ctx.fillStyle = '#0a0c0f';
    ctx.font = '9px Space Mono';
    ctx.textAlign = 'center';
    ctx.fillText('R', robotX, trackY + 4);

    // State info
    ctx.fillStyle = '#4a5a4a';
    ctx.font = '10px Space Mono';
    ctx.textAlign = 'left';
    ctx.fillText(`x = ${x.toFixed(2)}`, 16, 20);
    ctx.fillText(`ẋ = ${v.toFixed(2)}`, 16, 34);
    const K = getK();
    ctx.fillText(`K = [${K[0].toFixed(2)}, ${K[1].toFixed(2)}]`, 16, 48);
  }

  function drawSpring(ctx, x1, y1, x2, y2) {
    const n = 12, amp = 8;
    const len = x2 - x1;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    for (let i = 1; i <= n; i++) {
      const px = x1 + (len / n) * i;
      const py = y1 + (i % 2 === 0 ? -amp : amp);
      ctx.lineTo(px, py);
    }
    ctx.lineTo(x2, y2);
    ctx.strokeStyle = 'rgba(68,136,255,0.4)';
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  function updateMatrixDisplay() {
    const zeta = gp('zeta'), wn = gp('wn');
    const K = getK();
    const a11 = 0, a12 = 1;
    const a21 = -(wn * wn).toFixed(2), a22 = -(2 * zeta * wn).toFixed(2);
    document.getElementById('matrixDisplay').textContent =
      `A = [[${a11},  ${a12}     ],       ωₙ = ${wn.toFixed(2)},  ζ = ${zeta.toFixed(2)}
     [${a21}, ${a22}]]

B = [[0  ],       Closed-loop poles at:
     [ωₙ²]]       s = ${(-zeta * wn).toFixed(2)} ± ${(wn * Math.sqrt(Math.abs(1 - zeta * zeta))).toFixed(2)}j

K = [${K[0].toFixed(3)},  ${K[1].toFixed(3)}]   u = -K·[x; ẋ]`;
  }

  function initCharts() {
    if (phaseChart) phaseChart.destroy();
    if (stateChart) stateChart.destroy();

    phaseChart = new Chart(document.getElementById('phaseChart'), {
      type: 'scatter',
      data: { datasets: [{ data: [], borderColor: '#00ff88', backgroundColor: 'rgba(0,255,136,0.6)', pointRadius: 1.5, showLine: true, tension: 0.3 }] },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: { legend: { display: false } },
        scales: {
          x: { title: { display: true, text: 'position x', color: '#4a5a4a', font: { size: 10, family: 'Space Mono' } }, grid: { color: 'rgba(0,255,136,0.06)' }, ticks: { color: '#4a5a4a', font: { size: 9 }, maxTicksLimit: 6 } },
          y: { title: { display: true, text: 'velocity ẋ', color: '#4a5a4a', font: { size: 10, family: 'Space Mono' } }, grid: { color: 'rgba(0,255,136,0.06)' }, ticks: { color: '#4a5a4a', font: { size: 9 }, maxTicksLimit: 6 } }
        }
      }
    });

    stateChart = new Chart(document.getElementById('stateChart'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'x', data: [], borderColor: '#00ff88', pointRadius: 0, borderWidth: 1.5, tension: 0.3 },
          { label: 'ẋ', data: [], borderColor: '#ffcc00', pointRadius: 0, borderWidth: 1.5, tension: 0.3, borderDash: [4, 3] }
        ]
      },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: {
          legend: { display: true, labels: { color: '#8a9a8a', boxWidth: 10, font: { size: 10, family: 'Space Mono' } } }
        },
        scales: {
          x: { display: false },
          y: { grid: { color: 'rgba(0,255,136,0.06)' }, ticks: { color: '#4a5a4a', font: { size: 9 }, maxTicksLimit: 5 } }
        }
      }
    });
  }

  function updateCharts() {
    if (!phaseChart || !stateChart) return;
    const pts = xHistory.map((xi, i) => ({ x: xi, y: vHistory[i] }));
    phaseChart.data.datasets[0].data = pts;
    phaseChart.update('none');

    stateChart.data.labels = tHistory.map(tt => tt.toFixed(1));
    stateChart.data.datasets[0].data = xHistory;
    stateChart.data.datasets[1].data = vHistory;
    stateChart.update('none');
  }

  function loop() {
    step(); draw(); updateCharts();
    raf = requestAnimationFrame(loop);
  }

  function startSim() {
    running = true;
    document.getElementById('ssRunBtn').textContent = '⏸ PAUSE';
    loop();
  }

  function stopSim() {
    cancelAnimationFrame(raf);
    running = false;
    document.getElementById('ssRunBtn').textContent = '▶ RUN';
  }

  function bindSlider(id, outId, dec) {
    const el = document.getElementById(id);
    const out = document.getElementById(outId);
    el.addEventListener('input', () => {
      out.textContent = parseFloat(el.value).toFixed(dec);
      if (!running) { x = gp('x0'); v = gp('v0'); updateMatrixDisplay(); draw(); }
    });
  }

  function init() {
    bindSlider('zeta', 'zetaVal', 2);
    bindSlider('wn', 'wnVal', 1);
    bindSlider('gain', 'gainVal', 1);
    bindSlider('x0', 'x0Val', 0);
    bindSlider('v0', 'v0Val', 0);

    document.getElementById('ssRunBtn').addEventListener('click', () => {
      running ? stopSim() : startSim();
    });
    document.getElementById('ssResetBtn').addEventListener('click', () => {
      stopSim(); reset();
    });

    initCharts();
    reset();
  }

  window.addEventListener('resize', () => { if (!running) draw(); });

  return { init };
})();

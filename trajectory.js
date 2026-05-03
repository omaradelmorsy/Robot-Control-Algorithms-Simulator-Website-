
//  TRAJECTORY TRACKING : Pure Pursuit Algorithm

// Robot model: unicycle (x, y, heading θ)
// Control law: Pure Pursuit — find look-ahead point on path,
// compute curvature κ = 2*sin(α)/L, then v and ω

const Trajectory = (() => {
  const canvas = document.getElementById('trajCanvas');
  const ctx = canvas.getContext('2d');

  let raf, running = false;
  let dt = 0.016;

  // Robot state
  let rx, ry, rtheta;
  let rTrail = [];
  let distTraveled = 0;

  // Path
  let waypoints = [];
  let smoothPath = [];
  let closestIdx = 0;

  // Look-ahead point
  let laX = null, laY = null;
  let curvature = 0, headingErr = 0, cte = 0;

  // Metrics
  let cteHistory = [];
  let cteChart = null;

  const gp = id => parseFloat(document.getElementById(id).value);

  //  Default waypoints 
  function defaultWaypoints() {
    const cw = canvas.offsetWidth || 700, ch = 380;
    const cx = cw / 2, cy = ch / 2;
    return [
      { x: cx - 200, y: cy + 80 },
      { x: cx - 160, y: cy - 80 },
      { x: cx - 60, y: cy - 140 },
      { x: cx + 40, y: cy - 100 },
      { x: cx + 130, y: cy - 10 },
      { x: cx + 160, y: cy + 100 },
      { x: cx + 60, y: cy + 150 },
      { x: cx - 60, y: cy + 130 },
      { x: cx - 160, y: cy + 60 },
    ];
  }

  //  Catmull-Rom spline 
  function catmullRom(p0, p1, p2, p3, t) {
    const t2 = t * t, t3 = t2 * t;
    return {
      x: 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t +
        (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
        (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3),
      y: 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t +
        (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
        (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3)
    };
  }

  function buildSmoothPath() {
    if (waypoints.length < 2) { smoothPath = [...waypoints]; return; }
    const alpha = gp('smooth');
    smoothPath = [];
    const pts = [waypoints[0], ...waypoints, waypoints[waypoints.length - 1]];
    const steps = 30;
    for (let i = 1; i < pts.length - 2; i++) {
      for (let j = 0; j < steps; j++) {
        const t = j / steps;
        const p = catmullRom(pts[i - 1], pts[i], pts[i + 1], pts[i + 2], t);
        // Blending with straight line based on smoothing factor
        const straight = {
          x: pts[i].x + (pts[i + 1].x - pts[i].x) * t,
          y: pts[i].y + (pts[i + 1].y - pts[i].y) * t
        };
        smoothPath.push({
          x: p.x * alpha + straight.x * (1 - alpha),
          y: p.y * alpha + straight.y * (1 - alpha)
        });
      }
    }
    smoothPath.push(waypoints[waypoints.length - 1]);
  }

  //  Robot reset to first waypoint 
  function resetRobot() {
    closestIdx = 0;
    if (smoothPath.length >= 2) {
      rx = smoothPath[0].x; ry = smoothPath[0].y;
      const dx = smoothPath[1].x - smoothPath[0].x;
      const dy = smoothPath[1].y - smoothPath[0].y;
      rtheta = Math.atan2(dy, dx);
    } else if (waypoints.length > 0) {
      rx = waypoints[0].x; ry = waypoints[0].y; rtheta = 0;
    } else {
      rx = (canvas.offsetWidth || 700) / 2;
      ry = 190; rtheta = 0;
    }
    rTrail = [{ x: rx, y: ry }];
    distTraveled = 0; cteHistory = []; laX = null; laY = null;
    curvature = 0; headingErr = 0; cte = 0;
    updateMetrics();
    draw();
  }

  //  Finding closest point on path 
  function findClosest() {
    if (smoothPath.length === 0) return 0;
    let minD = Infinity, idx = closestIdx;
    const search = Math.min(smoothPath.length, closestIdx + 60);
    for (let i = closestIdx; i < search; i++) {
      const d = dist(rx, ry, smoothPath[i].x, smoothPath[i].y);
      if (d < minD) { minD = d; idx = i; }
    }
    return idx;
  }

  //  Finding look-ahead point 
  function findLookAhead(la) {
    for (let i = closestIdx; i < smoothPath.length; i++) {
      const d = dist(rx, ry, smoothPath[i].x, smoothPath[i].y);
      if (d >= la) return smoothPath[i];
    }
    return smoothPath[smoothPath.length - 1];
  }

  function dist(x1, y1, x2, y2) {
    return Math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2);
  }

  //  Simulation step 
  function step() {
    if (smoothPath.length < 2) return;
    const la = gp('la');
    const maxv = gp('maxsp');
    const L = gp('wheel');

    closestIdx = findClosest();
    const laPoint = findLookAhead(la);
    laX = laPoint.x; laY = laPoint.y;

    // Cross-track error: signed distance to closest segment
    const ci = closestIdx;
    if (ci < smoothPath.length - 1) {
      const seg = smoothPath[ci + 1];
      const dxs = seg.x - smoothPath[ci].x;
      const dys = seg.y - smoothPath[ci].y;
      const len = Math.sqrt(dxs * dxs + dys * dys) || 1;
      cte = Math.abs(((ry - smoothPath[ci].y) * dxs - (rx - smoothPath[ci].x) * dys) / len);
    }

    // Pure pursuit curvature
    const dx = laX - rx;
    const dy = laY - ry;
    const alpha = Math.atan2(dy, dx) - rtheta;
    curvature = 2 * Math.sin(alpha) / la;
    headingErr = alpha * (180 / Math.PI);

    // Unicycle kinematics
    // Speed: slow down when curvature is high
    const v = maxv * Math.max(0.2, 1 - Math.abs(curvature) * L * 0.5);
    const omega = v * curvature;

    const prevX = rx, prevY = ry;
    rx += v * Math.cos(rtheta) * dt * 60;
    ry += v * Math.sin(rtheta) * dt * 60;
    rtheta += omega * dt * 60;
    distTraveled += dist(prevX, prevY, rx, ry);

    rTrail.push({ x: rx, y: ry });
    if (rTrail.length > 600) rTrail.shift();

    cteHistory.push(cte);
    if (cteHistory.length > 300) cteHistory.shift();

    // Stopping near end
    const endPt = smoothPath[smoothPath.length - 1];
    if (dist(rx, ry, endPt.x, endPt.y) < 8 && closestIdx >= smoothPath.length - 10) {
      stopSim();
    }

    updateMetrics();
  }

  function updateMetrics() {
    document.getElementById('mCte').textContent = cte.toFixed(1);
    document.getElementById('mKappa').textContent = curvature.toFixed(3);
    document.getElementById('mHead').textContent = headingErr.toFixed(1);
    document.getElementById('mDist').textContent = Math.round(distTraveled);
    updateCteChart();
  }

  //  Draw 
  function draw() {
    const cw = canvas.offsetWidth || 700, ch = 380;
    canvas.width = cw; canvas.height = ch;
    ctx.clearRect(0, 0, cw, ch);
    ctx.fillStyle = '#0f1215';
    ctx.fillRect(0, 0, cw, ch);

    // Grid
    ctx.strokeStyle = 'rgba(0,255,136,0.04)';
    ctx.lineWidth = 1;
    for (let x = 0; x < cw; x += 40) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, ch); ctx.stroke(); }
    for (let y = 0; y < ch; y += 40) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(cw, y); ctx.stroke(); }

    if (waypoints.length === 0) {
      ctx.fillStyle = 'rgba(0,255,136,0.15)';
      ctx.font = '13px Space Mono';
      ctx.textAlign = 'center';
      ctx.fillText('Click to place waypoints', cw / 2, ch / 2);
      return;
    }

    // Smooth path
    if (smoothPath.length >= 2) {
      ctx.beginPath();
      ctx.moveTo(smoothPath[0].x, smoothPath[0].y);
      for (let i = 1; i < smoothPath.length; i++) ctx.lineTo(smoothPath[i].x, smoothPath[i].y);
      ctx.strokeStyle = '#4488ff';
      ctx.lineWidth = 1.5;
      ctx.globalAlpha = 0.4;
      ctx.setLineDash([5, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.globalAlpha = 1;
    }

    // Robot trail
    if (rTrail.length >= 2) {
      ctx.beginPath();
      ctx.moveTo(rTrail[0].x, rTrail[0].y);
      for (let i = 1; i < rTrail.length; i++) ctx.lineTo(rTrail[i].x, rTrail[i].y);
      ctx.strokeStyle = '#00ff88';
      ctx.lineWidth = 2;
      ctx.globalAlpha = 0.75;
      ctx.stroke();
      ctx.globalAlpha = 1;
    }

    // Waypoints
    waypoints.forEach((wp, i) => {
      ctx.beginPath();
      ctx.arc(wp.x, wp.y, 6, 0, Math.PI * 2);
      ctx.fillStyle = '#4488ff';
      ctx.shadowColor = '#4488ff';
      ctx.shadowBlur = 8;
      ctx.fill();
      ctx.shadowBlur = 0;
      ctx.fillStyle = '#4a5a4a';
      ctx.font = '9px Space Mono';
      ctx.textAlign = 'center';
      ctx.fillText((i + 1).toString(), wp.x, wp.y - 10);
    });

    // Look-ahead circle
    if (laX !== null) {
      const la = gp('la');
      ctx.beginPath();
      ctx.arc(rx, ry, la, 0, Math.PI * 2);
      ctx.strokeStyle = 'rgba(255,204,0,0.12)';
      ctx.lineWidth = 1;
      ctx.stroke();
      // Look-ahead line
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(laX, laY);
      ctx.strokeStyle = 'rgba(255,204,0,0.4)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
      // Look-ahead point
      ctx.beginPath();
      ctx.arc(laX, laY, 5, 0, Math.PI * 2);
      ctx.fillStyle = '#ffcc00';
      ctx.shadowColor = '#ffcc00';
      ctx.shadowBlur = 10;
      ctx.fill();
      ctx.shadowBlur = 0;
    }

    // Robot body
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(rtheta);
    ctx.fillStyle = '#00ff88';
    ctx.shadowColor = '#00ff88';
    ctx.shadowBlur = 16;
    ctx.fillRect(-10, -7, 20, 14);
    ctx.shadowBlur = 0;
    ctx.strokeStyle = '#0a0c0f';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(-10, -7, 20, 14);
    // Direction arrow
    ctx.fillStyle = '#0a0c0f';
    ctx.beginPath();
    ctx.moveTo(8, 0);
    ctx.lineTo(2, -4);
    ctx.lineTo(2, 4);
    ctx.fill();
    ctx.restore();
  }

  //  CTE Chart 
  function initCteChart() {
    if (cteChart) cteChart.destroy();
    cteChart = new Chart(document.getElementById('cteChart'), {
      type: 'line',
      data: {
        labels: Array(300).fill(''),
        datasets: [{
          data: Array(300).fill(0),
          borderColor: '#ffcc00',
          backgroundColor: 'rgba(255,204,0,0.07)',
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

  function updateCteChart() {
    if (!cteChart) return;
    const padded = [...Array(300 - cteHistory.length).fill(null), ...cteHistory];
    cteChart.data.datasets[0].data = padded;
    cteChart.update('none');
  }

  //  Animation 
  function loop() {
    step(); draw();
    raf = requestAnimationFrame(loop);
  }

  function startSim() {
    if (smoothPath.length < 2) return;
    running = true;
    document.getElementById('trajRunBtn').textContent = '⏸ PAUSE';
    document.getElementById('trajOverlay').textContent = 't = running';
    loop();
  }

  function stopSim() {
    cancelAnimationFrame(raf);
    running = false;
    document.getElementById('trajRunBtn').textContent = '▶ RUN';
  }

  function bindSlider(id, outId, dec) {
    const el = document.getElementById(id);
    const out = document.getElementById(outId);
    el.addEventListener('input', () => {
      out.textContent = parseFloat(el.value).toFixed(dec);
      if (id === 'smooth') { buildSmoothPath(); if (!running) draw(); }
    });
  }

  function init() {
    bindSlider('la', 'laVal', 0);
    bindSlider('maxsp', 'maxspVal', 1);
    bindSlider('wheel', 'wheelVal', 0);
    bindSlider('smooth', 'smoothVal', 2);

    // Canvas click  adding waypoints
    canvas.addEventListener('click', e => {
      if (running) return;
      const rect = canvas.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      waypoints.push({ x, y });
      document.getElementById('wpCount').textContent = waypoints.length;
      buildSmoothPath();
      resetRobot();
      document.getElementById('trajOverlay').textContent = `${waypoints.length} waypoint${waypoints.length > 1 ? 's' : ''} — click to add more`;
    });

    document.getElementById('clearWpBtn').addEventListener('click', () => {
      stopSim(); waypoints = []; smoothPath = [];
      document.getElementById('wpCount').textContent = 0;
      document.getElementById('trajOverlay').textContent = 'Click canvas to add waypoints';
      resetRobot();
    });

    document.getElementById('defaultWpBtn').addEventListener('click', () => {
      stopSim();
      waypoints = defaultWaypoints();
      document.getElementById('wpCount').textContent = waypoints.length;
      buildSmoothPath();
      resetRobot();
      document.getElementById('trajOverlay').textContent = `Demo path loaded — ${waypoints.length} waypoints`;
    });

    document.getElementById('trajRunBtn').addEventListener('click', () => {
      running ? stopSim() : startSim();
    });

    document.getElementById('trajResetBtn').addEventListener('click', () => {
      stopSim(); buildSmoothPath(); resetRobot();
      document.getElementById('trajOverlay').textContent = 'Reset — click Run to start';
    });

    initCteChart();
    resetRobot();
    draw();
  }

  window.addEventListener('resize', () => { if (!running) { buildSmoothPath(); resetRobot(); } });

  return { init };
})();

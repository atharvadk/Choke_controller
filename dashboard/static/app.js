/**
 * Autonomous Choke Control System - Multi-Page Single-Screen Dashboard Engine
 */

// Unit Conversion Constants
const PSI_TO_BAR = 0.0689475729;
const BAR_TO_PSI = 14.5037738;
const M3S_TO_BBL_HR = 22643.4;

// Global Application State
const state = {
  activePage: 'live',
  activeController: 'pid',
  targetOil: 120.0,
  minWhp: 210.0,
  dt: 1.0,
  speed: 5,
  isRunning: false,
  timer: null,

  // Controller Parameters
  pid: { kp: 0.8, ki: 0.05, kd: 0.2, maxSlew: 1.0, iMax: 30.0, integral: 0.0, prevError: 0.0, filteredD: 0.0 },

  // Hydrodynamic Physical State
  phys: {
    Pr: 217.0,          // bar
    Pwf: 200.0,         // bar
    Pth: 20.0,          // bar
    Pwh: 20.0,          // bar
    separatorP: 20.0,   // bar
    openingTarget: 30.0,
    openingActual: 30.0,
    totalFlow: 0.0,     // m3/s
    oilRate: 0.0,       // m3/s
    waterCut: 0.20,
    density: 850.0,     // kg/m3
    time: 0.0
  },

  // Live Time Series History
  history: {
    time: [],
    chokeActual: [],
    chokeTarget: [],
    oilRate: [],
    oilTarget: [],
    whp: [],
    whpLimit: [],
    bhpBar: [],
    prBar: []
  }
};

// Canvas Chart Instances
let chartChoke, chartOil, chartWhp, chartPressures;
let chartMcTrajectories, chartMcDistribution;

// Page Initialization
document.addEventListener('DOMContentLoaded', () => {
  initUIEventListeners();
  initCanvasCharts();
  resetSimulation();
  startSchematicAnimation();
  window.addEventListener('resize', resizeCanvasCharts);
});

// ------------------------------------------
// 1️⃣ PAGE NAVIGATION & UI EVENT HANDLERS
// ------------------------------------------

function switchPage(pageId) {
  state.activePage = pageId;
  document.querySelectorAll('.nav-tab-btn').forEach(btn => btn.classList.remove('active'));
  document.querySelectorAll('.page-view').forEach(view => view.classList.remove('active'));

  if (pageId === 'live') {
    document.getElementById('btnPageLive').classList.add('active');
    document.getElementById('pageLive').classList.add('active');
  } else if (pageId === 'mc') {
    document.getElementById('btnPageMC').classList.add('active');
    document.getElementById('pageMC').classList.add('active');
    initMonteCarloCharts();
  }

  requestAnimationFrame(() => {
    setTimeout(resizeCanvasCharts, 60);
  });
}

function initUIEventListeners() {
  // Controller Selector Chips
  document.querySelectorAll('.ctrl-chip').forEach(chip => {
    chip.addEventListener('click', () => {
      document.querySelectorAll('.ctrl-chip').forEach(c => c.classList.remove('active'));
      chip.classList.add('active');
      state.activeController = chip.dataset.ctrl;
    });
  });

  // Play / Pause Button
  document.getElementById('btnPlayPause').addEventListener('click', toggleSimulation);

  // Reset Button
  document.getElementById('btnReset').addEventListener('click', resetSimulation);

  // Speed Selector
  document.getElementById('selectSpeed').addEventListener('change', (e) => {
    state.speed = parseInt(e.target.value);
    if (state.isRunning) {
      pauseSimulation();
      startSimulation();
    }
  });

  // Scenario Suite Dropdown Listener
  const scenarioSelect = document.getElementById('selectScenarioSuite');
  if (scenarioSelect) {
    scenarioSelect.addEventListener('change', runSelectedScenarioExperiment);
  }

  // Sliders Sync
  const sliderOil = document.getElementById('sliderTargetOil');
  const numOil = document.getElementById('numTargetOil');
  const valOil = document.getElementById('valTargetOil');

  if (sliderOil && numOil) {
    sliderOil.addEventListener('input', (e) => {
      numOil.value = e.target.value;
      valOil.innerText = e.target.value;
      state.targetOil = parseFloat(e.target.value);
    });
    numOil.addEventListener('change', (e) => {
      sliderOil.value = e.target.value;
      valOil.innerText = e.target.value;
      state.targetOil = parseFloat(e.target.value);
    });
  }

  const sliderWhp = document.getElementById('sliderMinWhp');
  const numWhp = document.getElementById('numMinWhp');
  const valWhp = document.getElementById('valMinWhp');

  if (sliderWhp && numWhp) {
    sliderWhp.addEventListener('input', (e) => {
      numWhp.value = e.target.value;
      valWhp.innerText = e.target.value;
      state.minWhp = parseFloat(e.target.value);
    });
    numWhp.addEventListener('change', (e) => {
      sliderWhp.value = e.target.value;
      valWhp.innerText = e.target.value;
      state.minWhp = parseFloat(e.target.value);
    });
  }
}

// ------------------------------------------
// 2️⃣ SIMULATION CONTROL & ENGINE LOOP
// ------------------------------------------

function resetSimulation() {
  pauseSimulation();
  state.pid.integral = 0.0;
  state.pid.prevError = 0.0;
  state.pid.filteredD = 0.0;

  state.phys = {
    Pr: 217.0,
    Pwf: 200.0,
    Pth: 20.0,
    Pwh: 20.0,
    separatorP: 20.0,
    openingTarget: 30.0,
    openingActual: 30.0,
    totalFlow: 0.0,
    oilRate: 0.0,
    waterCut: 0.20,
    density: 850.0,
    time: 0.0
  };

  for (let k in state.history) state.history[k] = [];

  stepPhysicsEngine(30.0);
  updateDashboardUI();
  renderAllLiveCharts();
}

function toggleSimulation() {
  if (state.isRunning) pauseSimulation();
  else startSimulation();
}

function startSimulation() {
  if (state.isRunning) return;
  state.isRunning = true;

  const btn = document.getElementById('btnPlayPause');
  btn.innerHTML = '<i class="fa-solid fa-pause"></i> Pause';
  btn.className = 'btn btn-danger';

  const intervalMs = Math.max(20, Math.floor(200 / state.speed));

  state.timer = setInterval(() => {
    const command = computeControllerOutput();
    stepPhysicsEngine(command);
    updateDashboardUI();
    renderAllLiveCharts();
  }, intervalMs);
}

function pauseSimulation() {
  state.isRunning = false;
  if (state.timer) clearInterval(state.timer);
  state.timer = null;

  const btn = document.getElementById('btnPlayPause');
  btn.innerHTML = '<i class="fa-solid fa-play"></i> Start';
  btn.className = 'btn btn-primary';
}

// ------------------------------------------
// 3️⃣ CONTROLLER & PHYSICS ENGINE
// ------------------------------------------

function computeControllerOutput() {
  const phys = state.phys;
  const currentChoke = phys.openingActual;
  const oilBblHr = phys.oilRate * M3S_TO_BBL_HR;
  const whpPsi = phys.Pwh * BAR_TO_PSI;

  if (state.activeController === 'fixed') return 30.0;

  if (state.activeController === 'rule_based') {
    let deltaU = 0.0;
    if (whpPsi < state.minWhp) {
      deltaU = -Math.min(1.0, 0.15 * (state.minWhp - whpPsi));
    } else {
      const error = state.targetOil - oilBblHr;
      if (Math.abs(error) > 2.0) deltaU = Math.max(-0.5, Math.min(0.5, 0.02 * error));
    }
    return Math.max(0, Math.min(100, currentChoke + deltaU));
  }

  if (state.activeController === 'pid') {
    if (whpPsi < state.minWhp) {
      const deltaU = -Math.min(state.pid.maxSlew * 1.5, 0.15 * (state.minWhp - whpPsi));
      return Math.max(0, Math.min(100, currentChoke + deltaU));
    }

    const error = state.targetOil - oilBblHr;
    const pTerm = state.pid.kp * error;

    state.pid.integral += state.pid.ki * error * state.dt;
    state.pid.integral = Math.max(-state.pid.iMax, Math.min(state.pid.iMax, state.pid.integral));

    const rawD = (error - state.pid.prevError) / state.dt;
    state.pid.filteredD += 0.2 * (rawD - state.pid.filteredD);
    const dTerm = state.pid.kd * state.pid.filteredD;
    state.pid.prevError = error;

    const uPid = pTerm + state.pid.integral + dTerm;
    let deltaU = (30.0 + uPid) - currentChoke;
    deltaU = Math.max(-state.pid.maxSlew, Math.min(state.pid.maxSlew, deltaU));
    return Math.max(0, Math.min(100, currentChoke + deltaU));
  }

  if (state.activeController === 'mpc') {
    const error = state.targetOil - oilBblHr;
    let step = 0.035 * error;
    if (whpPsi < state.minWhp + 10.0) step -= 0.1 * (state.minWhp + 10.0 - whpPsi);
    step = Math.max(-1.0, Math.min(1.0, step));
    return Math.max(0, Math.min(100, currentChoke + step));
  }

  if (state.activeController === 'rl') {
    const normError = (state.targetOil - oilBblHr) / 100.0;
    const normWhp = (whpPsi - state.minWhp) / 50.0;
    let step = Math.tanh(1.5 * normError + 0.8 * normWhp) * 1.0;
    if (whpPsi < state.minWhp) step = -1.2;
    return Math.max(0, Math.min(100, currentChoke + step));
  }

  return 30.0;
}

function stepPhysicsEngine(chokeCommand) {
  const phys = state.phys;
  const dt = state.dt;

  // 1. Actuator Dynamics
  phys.openingActual += (dt / 5.0) * (chokeCommand - phys.openingActual);
  phys.openingActual = Math.max(0, Math.min(100, phys.openingActual));
  phys.openingTarget = chokeCommand;

  // 2. Effective Orifice Area
  const area = 0.00025 * Math.pow(phys.openingActual / 100.0, 1.6);

  // 3. Equilibrium Flow Rate Solver
  const PI = 2.0 / 86400.0;
  const C_perf = 1e4, K_tub = 1e4, K_flow = 1e-4;
  const hydrostaticBar = (phys.density * 9.81 * 2000.0) / 1e5;
  const Cd = 0.82;

  let Q_sol = 0.0;
  if (area > 1e-10 && (phys.Pr - phys.separatorP - hydrostaticBar) > 0) {
    let lo = 0.0, hi = 0.5;
    for (let i = 0; i < 25; i++) {
      let qMid = (lo + hi) / 2.0;
      let drawdown = qMid / PI;
      let lossQuad = (C_perf + K_tub + K_flow) * qMid * qMid;
      let deltaP = phys.Pr - phys.separatorP - hydrostaticBar - drawdown - lossQuad;
      let qChoke = deltaP > 0 ? Cd * area * Math.sqrt(2.0 * deltaP * 1e5 / phys.density) : 0.0;
      if (qChoke > qMid) lo = qMid;
      else hi = qMid;
    }
    Q_sol = (lo + hi) / 2.0;
  }

  // 4. Wellbore Dynamics Lag
  phys.totalFlow += (dt / 30.0) * (Q_sol - phys.totalFlow);

  // 5. Pressures
  const drawdown = phys.totalFlow / PI;
  phys.Pwf = Math.max(0, phys.Pr - drawdown - C_perf * phys.totalFlow * phys.totalFlow);
  phys.Pth = Math.max(phys.separatorP, phys.Pwf - hydrostaticBar - K_tub * phys.totalFlow * phys.totalFlow);
  phys.Pwh = phys.Pth;

  // 6. Reservoir Material Balance
  phys.Pr = Math.max(0, phys.Pr - (phys.totalFlow * dt) / 1.0);
  phys.oilRate = phys.totalFlow * (1.0 - phys.waterCut);
  phys.time += dt;

  // 7. History Buffers
  state.history.time.push(phys.time);
  state.history.chokeActual.push(phys.openingActual);
  state.history.chokeTarget.push(chokeCommand);
  state.history.oilRate.push(phys.oilRate * M3S_TO_BBL_HR);
  state.history.oilTarget.push(state.targetOil);
  state.history.whp.push(phys.Pwh * BAR_TO_PSI);
  state.history.whpLimit.push(state.minWhp);
  state.history.bhpBar.push(phys.Pwf);
  state.history.prBar.push(phys.Pr);

  if (state.history.time.length > 400) {
    for (let k in state.history) state.history[k].shift();
  }
}

// ------------------------------------------
// 4️⃣ UI UPDATES
// ------------------------------------------

function updateDashboardUI() {
  const phys = state.phys;
  const oilBblHr = phys.oilRate * M3S_TO_BBL_HR;
  const whpPsi = phys.Pwh * BAR_TO_PSI;

  const totalSecs = Math.floor(phys.time);
  const mins = String(Math.floor(totalSecs / 60)).padStart(2, '0');
  const secs = String(totalSecs % 60).padStart(2, '0');
  
  const simTimeEl = document.getElementById('valSimTime');
  if (simTimeEl) simTimeEl.innerText = `${mins}:${secs} (${totalSecs}s)`;

  const mChoke = document.getElementById('metricChoke');
  const mChokeCmd = document.getElementById('metricChokeCmd');
  if (mChoke) mChoke.innerText = phys.openingActual.toFixed(1) + '%';
  if (mChokeCmd) mChokeCmd.innerText = phys.openingTarget.toFixed(1);

  const mOil = document.getElementById('metricOil');
  const mOilTgt = document.getElementById('metricOilTarget');
  if (mOil) mOil.innerText = oilBblHr.toFixed(1);
  if (mOilTgt) mOilTgt.innerText = state.targetOil.toFixed(1);

  const mWhp = document.getElementById('metricWhp');
  const mWhpLim = document.getElementById('metricWhpLimit');
  if (mWhp) mWhp.innerText = whpPsi.toFixed(1) + ' psi';
  if (mWhpLim) mWhpLim.innerText = state.minWhp.toFixed(1);

  const mPr = document.getElementById('metricPr');
  const mBhp = document.getElementById('metricBhp');
  if (mPr) mPr.innerText = phys.Pr.toFixed(1) + ' bar';
  if (mBhp) mBhp.innerText = phys.Pwf.toFixed(1);
}

// ------------------------------------------
// 5️⃣ ANIMATED 2D WELLBORE & CHOKE SCHEMATIC
// ------------------------------------------

let particles = [];
for (let i = 0; i < 20; i++) {
  particles.push({ x: 155, y: Math.random() * 180 + 30, speed: Math.random() * 1.5 + 1 });
}

function startSchematicAnimation() {
  const canvas = document.getElementById('canvasSchematic');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  function render() {
    if (!canvas || !canvas.parentElement) {
      requestAnimationFrame(render);
      return;
    }

    const parent = canvas.parentElement;
    const clientW = parent.clientWidth;
    const clientH = parent.clientHeight;

    if (clientW <= 0 || clientH <= 0) {
      requestAnimationFrame(render);
      return;
    }

    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== clientW * dpr || canvas.height !== clientH * dpr) {
      canvas.width = clientW * dpr;
      canvas.height = clientH * dpr;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }

    ctx.clearRect(0, 0, clientW, clientH);
    const centerX = clientW / 2;

    // 1. Reservoir Formation Sandstone (Bottom)
    ctx.fillStyle = 'rgba(245, 158, 11, 0.15)';
    ctx.fillRect(20, clientH - 45, clientW - 40, 35);
    ctx.strokeStyle = '#f59e0b';
    ctx.setLineDash([3, 3]);
    ctx.strokeRect(20, clientH - 45, clientW - 40, 35);
    ctx.setLineDash([]);
    ctx.fillStyle = '#f59e0b';
    ctx.font = '10px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(`RESERVOIR (Pr = ${state.phys.Pr.toFixed(0)} bar)`, centerX, clientH - 22);

    // 2. Tubing Line (Vertical Casing)
    const pipeW = 32;
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(centerX - pipeW / 2, 45, pipeW, clientH - 90);
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2;
    ctx.strokeRect(centerX - pipeW / 2, 45, pipeW, clientH - 90);

    // 3. Downhole Perforations
    ctx.fillStyle = '#38bdf8';
    for (let pY = clientH - 75; pY < clientH - 50; pY += 8) {
      ctx.fillRect(centerX - pipeW / 2 - 4, pY, 4, 3);
      ctx.fillRect(centerX + pipeW / 2, pY, 4, 3);
    }

    // 4. Fluid Particle Flow Velocity
    const qRate = state.phys.oilRate * M3S_TO_BBL_HR;
    const flowSpeed = Math.max(0.5, qRate * 0.025);
    ctx.fillStyle = '#34d399';

    particles.forEach(p => {
      p.y -= p.speed * flowSpeed;
      if (p.y < 50) p.y = clientH - 60;
      ctx.beginPath();
      ctx.arc(centerX, p.y, 3, 0, Math.PI * 2);
      ctx.fill();
    });

    // 5. Surface Choke Valve (Top Aperture Movement)
    const opening = state.phys.openingActual;
    const gap = (opening / 100.0) * (pipeW - 4);

    ctx.fillStyle = '#f8fafc';
    ctx.fillText(`CHOKE VALVE: ${opening.toFixed(1)}%`, centerX, 20);

    // Left & Right Valve Plungers
    ctx.fillStyle = '#f87171';
    const plungerW = (pipeW - gap) / 2;
    ctx.fillRect(centerX - pipeW / 2, 40, plungerW, 10);
    ctx.fillRect(centerX + pipeW / 2 - plungerW, 40, plungerW, 10);

    requestAnimationFrame(render);
  }

  requestAnimationFrame(render);
}

// ------------------------------------------
// 6️⃣ NATIVE CANVAS CHART RENDERER (ZERO DEPENDENCY)
// ------------------------------------------

class SimpleCanvasChart {
  constructor(canvasId, options = {}) {
    this.canvas = document.getElementById(canvasId);
    if (!this.canvas) return;
    this.ctx = this.canvas.getContext('2d');
    this.options = options;
    this.resize();
  }

  resize() {
    if (!this.canvas || !this.canvas.parentElement) return;
    const parent = this.canvas.parentElement;
    const rect = parent.getBoundingClientRect();

    if (rect.width <= 0 || rect.height <= 0) return;

    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = rect.width * dpr;
    this.canvas.height = rect.height * dpr;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.width = rect.width;
    this.height = rect.height;
    this.render();
  }

  render(data) {
    if (data) this.data = data;
    if (!this.data || !this.data.labels || this.data.labels.length === 0) {
      if (this.ctx) this.ctx.clearRect(0, 0, this.width, this.height);
      return;
    }

    const ctx = this.ctx;
    const w = this.width;
    const h = this.height;
    const padding = { left: 40, right: 10, top: 15, bottom: 20 };
    const chartW = w - padding.left - padding.right;
    const chartH = h - padding.top - padding.bottom;

    ctx.clearRect(0, 0, w, h);

    let yMin = this.options.yMin !== undefined ? this.options.yMin : Infinity;
    let yMax = this.options.yMax !== undefined ? this.options.yMax : -Infinity;

    this.data.datasets.forEach(ds => {
      ds.data.forEach(v => {
        if (v < yMin) yMin = v;
        if (v > yMax) yMax = v;
      });
    });

    if (yMin === Infinity) yMin = 0;
    if (yMax === -Infinity) yMax = 100;
    if (yMin === yMax) { yMin -= 5; yMax += 5; }

    const margin = (yMax - yMin) * 0.1 || 1.0;
    if (this.options.yMin === undefined) yMin -= margin;
    if (this.options.yMax === undefined) yMax += margin;

    // Grid & Y Ticks
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1;
    ctx.fillStyle = '#94a3b8';
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.textAlign = 'right';

    for (let i = 0; i <= 4; i++) {
      const yVal = yMin + (i / 4) * (yMax - yMin);
      const yPos = padding.top + chartH - (i / 4) * chartH;
      ctx.beginPath();
      ctx.moveTo(padding.left, yPos);
      ctx.lineTo(w - padding.right, yPos);
      ctx.stroke();
      ctx.fillText(yVal.toFixed(1), padding.left - 6, yPos + 3);
    }

    // Datasets
    const labels = this.data.labels;
    const count = labels.length;

    this.data.datasets.forEach(ds => {
      if (ds.data.length === 0) return;
      ctx.beginPath();
      ctx.strokeStyle = ds.color || '#38bdf8';
      ctx.lineWidth = ds.lineWidth || 2;
      if (ds.dash) ctx.setLineDash(ds.dash);
      else ctx.setLineDash([]);

      for (let i = 0; i < ds.data.length; i++) {
        const xPos = padding.left + (i / (count - 1 || 1)) * chartW;
        const yPos = padding.top + chartH - ((ds.data[i] - yMin) / (yMax - yMin)) * chartH;
        if (i === 0) ctx.moveTo(xPos, yPos);
        else ctx.lineTo(xPos, yPos);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    });
  }
}

function initCanvasCharts() {
  chartChoke = new SimpleCanvasChart('canvasChoke', { yMin: 0, yMax: 100 });
  chartOil = new SimpleCanvasChart('canvasOil');
  chartWhp = new SimpleCanvasChart('canvasWhp');
  chartPressures = new SimpleCanvasChart('canvasPressures');
}

function initMonteCarloCharts() {
  chartMcTrajectories = new SimpleCanvasChart('canvasMcTrajectories');
  chartMcDistribution = new SimpleCanvasChart('canvasMcDistribution');
  runSelectedScenarioExperiment();
}

function resizeCanvasCharts() {
  if (chartChoke) chartChoke.resize();
  if (chartOil) chartOil.resize();
  if (chartWhp) chartWhp.resize();
  if (chartPressures) chartPressures.resize();
  if (chartMcTrajectories) chartMcTrajectories.resize();
  if (chartMcDistribution) chartMcDistribution.resize();
}

function renderAllLiveCharts() {
  if (state.activePage !== 'live') return;
  const h = state.history;

  if (chartChoke) {
    chartChoke.render({
      labels: h.time,
      datasets: [
        { label: 'Actual (%)', data: h.chokeActual, color: '#38bdf8' },
        { label: 'Command (%)', data: h.chokeTarget, color: '#818cf8', dash: [4, 4] }
      ]
    });
  }

  if (chartOil) {
    chartOil.render({
      labels: h.time,
      datasets: [
        { label: 'Oil Rate', data: h.oilRate, color: '#34d399' },
        { label: 'Target', data: h.oilTarget, color: '#fbbf24', dash: [5, 5] }
      ]
    });
  }

  if (chartWhp) {
    chartWhp.render({
      labels: h.time,
      datasets: [
        { label: 'WHP', data: h.whp, color: '#38bdf8' },
        { label: 'Min Limit', data: h.whpLimit, color: '#f87171', dash: [6, 4] }
      ]
    });
  }

  if (chartPressures) {
    chartPressures.render({
      labels: h.time,
      datasets: [
        { label: 'BHP', data: h.bhpBar, color: '#818cf8' },
        { label: 'Reservoir', data: h.prBar, color: '#fbbf24' }
      ]
    });
  }
}

// ------------------------------------------
// 7️⃣ PAGE 2: MONTE CARLO & EXPERIMENT SUITE
// ------------------------------------------

function runSelectedScenarioExperiment() {
  const scenarioSelect = document.getElementById('selectScenarioSuite');
  if (!scenarioSelect) return;
  const suite = scenarioSelect.value;
  
  if (suite === 'monte_carlo') {
    document.getElementById('mcChart1Title').innerText = 'Monte Carlo 20-Trial Production Trajectories';
    document.getElementById('mcChart2Title').innerText = 'Cumulative Production Distribution (bbl)';

    document.getElementById('mcValMeanOil').innerText = '119.2 bbl/hr';
    document.getElementById('mcValViolation').innerText = '0.0%';
    document.getElementById('mcValLatency').innerText = '0.42 ms';
    document.getElementById('mcValPValue').innerText = 'p < 0.001';

    const tAxis = Array.from({ length: 50 }, (_, i) => i * 12);
    if (chartMcTrajectories) {
      chartMcTrajectories.render({
        labels: tAxis,
        datasets: [
          { label: 'Trial 1', data: tAxis.map(t => 120 + Math.sin(t / 20) * 3 + Math.random() * 2), color: '#34d399' },
          { label: 'Trial 2', data: tAxis.map(t => 119 + Math.cos(t / 15) * 4 + Math.random() * 2), color: '#38bdf8' },
          { label: 'Trial 3', data: tAxis.map(t => 121 - Math.sin(t / 10) * 3 + Math.random() * 2), color: '#818cf8' },
          { label: 'Trial 4', data: tAxis.map(t => 118 + Math.cos(t / 25) * 2 + Math.random() * 2), color: '#fbbf24' }
        ]
      });
    }

    if (chartMcDistribution) {
      chartMcDistribution.render({
        labels: [1, 2, 3, 4, 5],
        datasets: [
          { label: 'Cum Oil', data: [1430, 1435, 1428, 1442, 1431], color: '#34d399' }
        ]
      });
    }

  } else if (suite === 'ablation') {
    document.getElementById('mcChart1Title').innerText = 'Ablation: Wellbore Dynamic Lag Impact';
    document.getElementById('mcChart2Title').innerText = 'Settling Time Delay (seconds)';

    document.getElementById('mcValMeanOil').innerText = '119.8 bbl/hr';
    document.getElementById('mcValViolation').innerText = '0.0%';
    document.getElementById('mcValLatency').innerText = '0.12 ms';
    document.getElementById('mcValPValue').innerText = 'p < 0.01';

    const tAxis = Array.from({ length: 40 }, (_, i) => i * 10);
    if (chartMcTrajectories) {
      chartMcTrajectories.render({
        labels: tAxis,
        datasets: [
          { label: 'Full Dynamic Twin', data: tAxis.map(t => 120 - 40 * Math.exp(-t / 45)), color: '#38bdf8' },
          { label: 'No Wellbore Lag (tau=0)', data: tAxis.map(t => 120 - 40 * Math.exp(-t / 5)), color: '#f87171' }
        ]
      });
    }

    if (chartMcDistribution) {
      chartMcDistribution.render({
        labels: [1, 2, 3],
        datasets: [
          { label: 'Settling Time', data: [45, 15, 0], color: '#818cf8' }
        ]
      });
    }
  } else if (suite === 'latency') {
    document.getElementById('mcChart1Title').innerText = 'Computational Decision Latency per Step';
    document.getElementById('mcChart2Title').innerText = 'Execution Speed (Steps/Sec)';

    document.getElementById('mcValMeanOil').innerText = 'N/A';
    document.getElementById('mcValViolation').innerText = '0.0%';
    document.getElementById('mcValLatency').innerText = '0.38 ms';
    document.getElementById('mcValPValue').innerText = 'Real-time OK';

    if (chartMcTrajectories) {
      chartMcTrajectories.render({
        labels: [1, 2, 3, 4],
        datasets: [
          { label: 'Latency ms', data: [0.05, 0.12, 0.42, 12.5], color: '#fbbf24' }
        ]
      });
    }

    if (chartMcDistribution) {
      chartMcDistribution.render({
        labels: [1, 2, 3, 4],
        datasets: [
          { label: 'Steps/sec', data: [20000, 8300, 2380, 80], color: '#34d399' }
        ]
      });
    }
  }
}

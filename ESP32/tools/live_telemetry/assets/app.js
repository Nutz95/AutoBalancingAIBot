const chartDefs = {
  pitch: {
    title: 'Pitch / command',
    axisHints: [
      { text: 'Left axis: pitch / PID input (blue-violet)', color: '#4ea1ff' },
      { text: 'Right axis: command / steer (green-orange)', color: '#24c37b' },
    ],
    scaleConfig: {
      y: { title: 'Pitch (deg)', color: '#4ea1ff', pad: 0.12, shrink: 0.18 },
      y1: { title: 'Command', color: '#24c37b', min: -1.1, max: 1.1 },
    },
    series: [
      { key: 'pitch_deg', label: 'Pitch (deg)', color: '#4ea1ff', axis: 'y' },
      { key: 'pid_in_deg', label: 'PID input (deg)', color: '#9d7dff', axis: 'y' },
      { key: 'cmd', label: 'Throttle cmd', color: '#24c37b', axis: 'y1' },
      { key: 'steer', label: 'Steer', color: '#ff8f5a', axis: 'y1' },
    ],
  },
  lqr: {
    title: 'LQR decomposition',
    axisHints: [{ text: 'Left axis: LQR terms', color: '#4ea1ff' }],
    scaleConfig: { y: { title: 'LQR terms', color: '#4ea1ff', pad: 0.14, shrink: 0.18 } },
    series: [
      { key: 'lqr_angle', label: 'Angle', color: '#4ea1ff' },
      { key: 'lqr_gyro', label: 'Gyro', color: '#f0b84b' },
      { key: 'lqr_dist', label: 'Distance', color: '#ff6b6b' },
      { key: 'lqr_speed', label: 'Speed', color: '#24c37b' },
    ],
  },
  timing: {
    title: 'Loop rate & profiling',
    axisHints: [
      { text: 'Left axis: loop Hz (red/pink)', color: '#ff6b6b' },
      { text: 'Right axis: profiling us (green/yellow/white)', color: '#dbe7ff' },
    ],
    scaleConfig: {
      y: { title: 'Loop Hz', color: '#ff6b6b', pad: 0.06, shrink: 0.07, quantileMin: 0.08, quantileMax: 0.96 },
      y1: { title: 'Profiling (us)', color: '#dbe7ff', pad: 0.10, shrink: 0.08, quantileMin: 0.02, quantileMax: 0.96 },
    },
    series: [
      { key: 'loop_freq_hz_ema', label: 'Loop Hz (EMA)', color: '#ff6b6b', axis: 'y' },
      { key: 'loop_freq_hz', label: 'Loop Hz raw', color: '#ff98a1', axis: 'y', dashed: true },
      { key: 'prof_f', label: 'Fusion us', color: '#24c37b', axis: 'y1' },
      { key: 'prof_l', label: 'LQR us', color: '#f0b84b', axis: 'y1' },
      { key: 'prof_t', label: 'Total us', color: '#dbe7ff', axis: 'y1' },
    ],
  },
  cpu: {
    title: 'CPU load',
    axisHints: [{ text: 'Left axis: CPU load %', color: '#4ea1ff' }],
    scaleConfig: { y: { title: 'CPU %', color: '#4ea1ff', min: 0, max: 100 } },
    series: [
      { key: 'cpu0_pct', label: 'Core0 %', color: '#4ea1ff' },
      { key: 'cpu1_pct', label: 'Core1 %', color: '#ff6b6b' },
    ],
  },
  rpm: {
    title: 'Motor RPM (RS485)',
    axisHints: [{ text: 'Left axis: motor RPM', color: '#4ea1ff' }],
    scaleConfig: { y: { title: 'RPM', color: '#4ea1ff', pad: 0.10, shrink: 0.14 } },
    series: [
      { key: 'motor_rpm_l', label: 'Left RPM', color: '#4ea1ff' },
      { key: 'motor_rpm_r', label: 'Right RPM', color: '#ff6b6b' },
    ],
  },
  step: {
    title: 'Applied STEP frequency',
    axisHints: [{ text: 'Left axis: applied STEP Hz', color: '#4ea1ff' }],
    scaleConfig: { y: { title: 'STEP Hz', color: '#4ea1ff', pad: 0.10, shrink: 0.14 } },
    series: [
      { key: 'step_hz_l', label: 'Left STEP Hz', color: '#4ea1ff' },
      { key: 'step_hz_r', label: 'Right STEP Hz', color: '#ff6b6b' },
    ],
  },
  bus: {
    title: 'Bus / ACK / encoder age',
    axisHints: [
      { text: 'Left axis: encoder age ms (white, hidden when idle)', color: '#dbe7ff' },
      { text: 'Right axis: bus / ACK us', color: '#4ea1ff' },
    ],
    scaleConfig: {
      y: { title: 'Encoder age (ms)', color: '#dbe7ff', min: 0, max: 300 },
      y1: { title: 'Bus / ACK (us)', color: '#4ea1ff', pad: 0.12, shrink: 0.12, quantileMin: 0.02, quantileMax: 0.97 },
    },
    series: [
      { key: 'bus_latency_left_us', label: 'Bus L us', color: '#4ea1ff', axis: 'y1' },
      { key: 'bus_latency_right_us', label: 'Bus R us', color: '#ff6b6b', axis: 'y1' },
      { key: 'ack_pending_left_us', label: 'ACK L us', color: '#24c37b', axis: 'y1' },
      { key: 'ack_pending_right_us', label: 'ACK R us', color: '#f0b84b', axis: 'y1' },
      { key: 'encoder_age_plot_ms', label: 'Encoder age ms', color: '#dbe7ff', axis: 'y' },
    ],
  },
  imu: {
    title: 'Raw IMU axes',
    axisHints: [{ text: 'Left axis: raw IMU', color: '#4ea1ff' }],
    scaleConfig: { y: { title: 'IMU raw', color: '#4ea1ff', pad: 0.14, shrink: 0.16 } },
    series: [
      { key: 'gx', label: 'Gyro X', color: '#4ea1ff' },
      { key: 'gy', label: 'Gyro Y', color: '#ff8f5a' },
      { key: 'gz', label: 'Gyro Z', color: '#9d7dff' },
      { key: 'ax', label: 'Accel X', color: '#24c37b' },
      { key: 'ay', label: 'Accel Y', color: '#f0b84b' },
      { key: 'az', label: 'Accel Z', color: '#dbe7ff' },
    ],
  },
  mixing: {
    title: 'Command mixing / saturation',
    axisHints: [{ text: 'Left axis: commands / wheel mix', color: '#dbe7ff' }],
    scaleConfig: { y: { title: 'Mix / command', color: '#dbe7ff', pad: 0.14, shrink: 0.16 } },
    series: [
      { key: 'cmd_raw', label: 'cmd raw', color: '#dbe7ff' },
      { key: 'cmd_final', label: 'cmd final', color: '#24c37b' },
      { key: 'left_preclip', label: 'left preclip', color: '#4ea1ff' },
      { key: 'right_preclip', label: 'right preclip', color: '#ff6b6b' },
      { key: 'left_postclip', label: 'left postclip', color: '#4ea1ff', dashed: true },
      { key: 'right_postclip', label: 'right postclip', color: '#ff6b6b', dashed: true },
    ],
  },
  nav: {
    title: 'Navigation / odometry analysis',
    axisHints: [
      { text: 'Left axis: relative encoder position', color: '#4ea1ff' },
      { text: 'Right axis: velocity / steer / asymmetry', color: '#24c37b' },
    ],
    scaleConfig: {
      y: { title: 'Relative encoder ticks', color: '#4ea1ff', pad: 0.14, shrink: 0.14 },
      y1: { title: 'Velocity / steer', color: '#24c37b', pad: 0.14, shrink: 0.14 },
    },
    series: [
      { key: 'enc_avg_rel', label: 'Forward dist proxy', color: '#4ea1ff', axis: 'y' },
      { key: 'enc_diff_rel', label: 'Turn diff proxy', color: '#ff8f5a', axis: 'y' },
      { key: 'enc_vel_proxy', label: 'Forward vel proxy', color: '#24c37b', axis: 'y1' },
      { key: 'rpm_diff', label: 'RPM diff', color: '#f0b84b', axis: 'y1' },
      { key: 'steer', label: 'Steer cmd', color: '#dbe7ff', axis: 'y1', dashed: true },
    ],
  },
  fft: {
    title: 'Live FFT / oscillations',
    source: 'fft',
    axisHints: [
      { text: 'X axis: frequency (Hz)', color: '#4ea1ff' },
      { text: 'Y axis: relative magnitude', color: '#dbe7ff' },
    ],
    scaleConfig: { y: { title: 'Magnitude', color: '#dbe7ff', pad: 0.18, shrink: 0.18, quantileMin: 0.0, quantileMax: 1.0 } },
    series: [
      { key: 'pitch_mag', label: 'Pitch FFT', color: '#4ea1ff' },
      { key: 'cmd_mag', label: 'Command FFT', color: '#24c37b' },
      { key: 'rpm_mag', label: 'RPM FFT', color: '#ff8f5a' },
    ],
  },
};

const defaultCharts = Object.keys(chartDefs);
const charts = new Map();
const axisState = new Map();
const visible = new Set(defaultCharts);
let refreshHandle = null;
let refreshMs = 200;

function metric(name, value) {
  return `<div class="metric"><div class="label">${name}</div><div class="value">${value}</div></div>`;
}

function buildSidebar() {
  const list = document.getElementById('chartList');
  list.innerHTML = '';
  for (const [key, def] of Object.entries(chartDefs)) {
    const item = document.createElement('label');
    item.className = 'check-item';
    item.innerHTML = `<input type="checkbox" ${visible.has(key) ? 'checked' : ''} data-key="${key}"> <span>${def.title}</span>`;
    item.querySelector('input').addEventListener('change', (ev) => {
      if (ev.target.checked) visible.add(key); else visible.delete(key);
      rebuildCharts();
    });
    list.appendChild(item);
  }
}

function getAxisConfig(def, axisId) {
  return (def.scaleConfig && def.scaleConfig[axisId]) || {};
}

function chartOptions(def) {
  const isFft = def.source === 'fft';
  const leftCfg = getAxisConfig(def, 'y');
  const rightCfg = getAxisConfig(def, 'y1');
  return {
    responsive: true,
    animation: false,
    maintainAspectRatio: false,
    interaction: { mode: 'nearest', intersect: false },
    scales: {
      x: {
        type: 'linear',
        title: { display: true, text: isFft ? 'Frequency (Hz)' : 'Time (s)', color: '#9ab0d6' },
        ticks: { color: '#9ab0d6' },
        grid: { color: 'rgba(154,176,214,0.12)' },
        min: isFft ? 0 : -10,
        max: isFft ? 25 : 0,
      },
      y: {
        ticks: { color: leftCfg.color || '#9ab0d6' },
        grid: { color: 'rgba(154,176,214,0.12)' },
        title: leftCfg.title ? { display: true, text: leftCfg.title, color: leftCfg.color || '#9ab0d6' } : undefined,
        min: Number.isFinite(leftCfg.min) ? leftCfg.min : undefined,
        max: Number.isFinite(leftCfg.max) ? leftCfg.max : undefined,
      },
      y1: {
        position: 'right',
        display: def.series.some((s) => (s.axis || 'y') === 'y1'),
        ticks: { color: rightCfg.color || '#9ab0d6' },
        grid: { drawOnChartArea: false },
        title: rightCfg.title ? { display: true, text: rightCfg.title, color: rightCfg.color || '#9ab0d6' } : undefined,
        min: Number.isFinite(rightCfg.min) ? rightCfg.min : undefined,
        max: Number.isFinite(rightCfg.max) ? rightCfg.max : undefined,
      },
    },
    plugins: {
      legend: { labels: { color: '#dbe7ff', boxWidth: 10 } },
      tooltip: { enabled: true },
    },
  };
}

function createChartCard(key, def) {
  const card = document.createElement('section');
  card.className = 'card';
  card.dataset.key = key;
  const axisHints = (def.axisHints || []).map((hint) => (
    `<span class="axis-hint"><span class="axis-swatch" style="background:${hint.color}"></span>${hint.text}</span>`
  )).join('');
  card.innerHTML = `<h3>${def.title}</h3>${axisHints ? `<div class="axis-hints">${axisHints}</div>` : ''}<canvas></canvas>`;
  const ctx = card.querySelector('canvas');
  const datasets = def.series.map((s) => ({
    label: s.label,
    data: [],
    parsing: false,
    borderColor: s.color,
    backgroundColor: s.color,
    borderWidth: 1.5,
    pointRadius: 0,
    tension: 0.1,
    yAxisID: s.axis || 'y',
    borderDash: s.dashed ? [6, 4] : [],
  }));
  const chart = new Chart(ctx, {
    type: 'line',
    data: { datasets },
    options: chartOptions(def),
  });
  charts.set(key, chart);
  return card;
}

function rebuildCharts() {
  const grid = document.getElementById('chartGrid');
  for (const chart of charts.values()) chart.destroy();
  charts.clear();
  grid.innerHTML = '';
  for (const [key, def] of Object.entries(chartDefs)) {
    if (!visible.has(key)) continue;
    grid.appendChild(createChartCard(key, def));
  }
  buildSidebar();
}

function quantile(values, q) {
  if (!values.length) return null;
  const sorted = [...values].sort((a, b) => a - b);
  const pos = (sorted.length - 1) * q;
  const base = Math.floor(pos);
  const rest = pos - base;
  const next = sorted[Math.min(base + 1, sorted.length - 1)];
  return sorted[base] + (next - sorted[base]) * rest;
}

function computeAxisRange(values, cfg) {
  const finite = values.filter((v) => Number.isFinite(v));
  if (!finite.length) return null;
  let min = quantile(finite, cfg.quantileMin ?? 0.02);
  let max = quantile(finite, cfg.quantileMax ?? 0.98);
  if (min == null || max == null) return null;
  if (min === max) {
    const widen = Math.max(1, Math.abs(min) * 0.1);
    min -= widen;
    max += widen;
  }
  const span = Math.max(1e-6, max - min);
  const pad = cfg.pad ?? 0.10;
  return { min: min - span * pad, max: max + span * pad };
}

function blendAxisRange(key, axisId, target, cfg) {
  const stateKey = `${key}:${axisId}`;
  const current = axisState.get(stateKey);
  if (!current) {
    axisState.set(stateKey, target);
    return target;
  }
  const shrink = cfg.shrink ?? 0.16;
  const next = {
    min: target.min < current.min ? target.min : current.min + (target.min - current.min) * shrink,
    max: target.max > current.max ? target.max : current.max + (target.max - current.max) * shrink,
  };
  axisState.set(stateKey, next);
  return next;
}

function updateAxisRanges(chart, key, def) {
  ['y', 'y1'].forEach((axisId) => {
    const axisCfg = getAxisConfig(def, axisId);
    if (Number.isFinite(axisCfg.min) || Number.isFinite(axisCfg.max)) {
      if (Number.isFinite(axisCfg.min)) chart.options.scales[axisId].min = axisCfg.min;
      if (Number.isFinite(axisCfg.max)) chart.options.scales[axisId].max = axisCfg.max;
      return;
    }
    const values = chart.data.datasets
      .filter((dataset) => (dataset.yAxisID || 'y') === axisId)
      .flatMap((dataset) => dataset.data.map((pt) => pt.y));
    const target = computeAxisRange(values, axisCfg);
    if (!target) return;
    const blended = blendAxisRange(key, axisId, target, axisCfg);
    chart.options.scales[axisId].min = blended.min;
    chart.options.scales[axisId].max = blended.max;
  });
}

function updateXAxis(chart, def, payload) {
  if (def.source === 'fft') {
    chart.options.scales.x.min = 0;
    chart.options.scales.x.max = Math.max(6, Number(payload.fft_meta?.x_max_hz || 12));
  } else {
    chart.options.scales.x.min = -Number(payload.window_sec || 10);
    chart.options.scales.x.max = 0;
  }
}

function updateMetrics(latest, statusText) {
  const grid = document.getElementById('metricGrid');
  if (!latest) {
    grid.innerHTML = metric('Packets', '0') + metric('Loop Hz', '—') + metric('Pitch', '—') + metric('CPU1', '—');
    document.getElementById('status').textContent = statusText || 'Waiting for telemetry…';
    return;
  }
  grid.innerHTML = [
    metric('Packets', latest.packet_count ?? '—'),
    metric('Loop Hz', Number(latest.loop_freq_hz_ema || latest.loop_freq_hz || 0).toFixed(1)),
    metric('Pitch', Number(latest.pitch_deg || 0).toFixed(2) + '°'),
    metric('CPU0', Number(latest.cpu0_pct || 0).toFixed(1) + '%'),
    metric('CPU1', Number(latest.cpu1_pct || 0).toFixed(1) + '%'),
    metric('RPM L/R', `${Number(latest.motor_rpm_l || 0).toFixed(0)} / ${Number(latest.motor_rpm_r || 0).toFixed(0)}`),
    metric('STEP L/R', `${Number(latest.step_hz_l || 0).toFixed(0)} / ${Number(latest.step_hz_r || 0).toFixed(0)}`),
    metric('Encoder age', latest.motion_active ? (Number(latest.last_encoder_age_ms || 0).toFixed(0) + ' ms') : 'idle'),
    metric('Balance activity', latest.motion_active ? 'active' : 'idle'),
  ].join('');
  document.getElementById('status').textContent = statusText;
}

function buildSeriesPoints(def, payload, series) {
  if (def.source === 'fft') {
    return (payload.fft || [])
      .map((row) => ({ x: row.freq_hz, y: row[series.key] }))
      .filter((pt) => Number.isFinite(pt.y));
  }
  return (payload.samples || [])
    .map((s) => ({ x: s.t_s, y: s[series.key] }))
    .filter((pt) => Number.isFinite(pt.y));
}

function updateCharts(payload) {
  for (const [key, chart] of charts.entries()) {
    const def = chartDefs[key];
    def.series.forEach((series, idx) => {
      chart.data.datasets[idx].data = buildSeriesPoints(def, payload, series);
    });
    updateXAxis(chart, def, payload);
    updateAxisRanges(chart, key, def);
    chart.update('none');
  }
}

async function refresh() {
  try {
    const resp = await fetch('/api/state', { cache: 'no-store' });
    const payload = await resp.json();
    document.getElementById('windowLabel').textContent = payload.window_sec;
    updateMetrics(payload.latest, payload.status);
    updateCharts(payload);
  } catch {
    document.getElementById('status').textContent = 'Dashboard disconnected';
  }
}

function scheduleRefresh() {
  if (refreshHandle) clearInterval(refreshHandle);
  refreshHandle = setInterval(refresh, refreshMs);
  refresh();
}

document.getElementById('allBtn').addEventListener('click', () => {
  Object.keys(chartDefs).forEach((k) => visible.add(k));
  rebuildCharts();
});

document.getElementById('noneBtn').addEventListener('click', () => {
  visible.clear();
  rebuildCharts();
});

document.getElementById('refreshSel').addEventListener('change', (ev) => {
  refreshMs = Number(ev.target.value);
  scheduleRefresh();
});

rebuildCharts();
scheduleRefresh();

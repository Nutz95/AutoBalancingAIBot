import { chartDefs, defaultCharts } from './chart-definitions.js';

function getAxisConfig(def, axisId) {
  return (def.scaleConfig && def.scaleConfig[axisId]) || {};
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

function buildSeriesPoints(def, payload, series) {
  if (def.source === 'fft') {
    return (payload.fft || [])
      .map((row) => ({ x: row.freq_hz, y: row[series.key] }))
      .filter((pt) => Number.isFinite(pt.y));
  }
  return (payload.samples || [])
    .map((sample) => ({ x: sample.t_s, y: sample[series.key] }))
    .filter((pt) => Number.isFinite(pt.y));
}

export class ChartManager {
  constructor(chartGridElement, chartListElement) {
    this._chartGridElement = chartGridElement;
    this._chartListElement = chartListElement;
    this._charts = new Map();
    this._axisState = new Map();
    this._visible = new Set(defaultCharts);
  }

  build() {
    this._destroyCharts();
    this._chartGridElement.innerHTML = '';
    for (const [key, def] of Object.entries(chartDefs)) {
      if (!this._visible.has(key)) continue;
      this._chartGridElement.appendChild(this._createChartCard(key, def));
    }
    this._buildSidebar();
  }

  showAll() {
    Object.keys(chartDefs).forEach((key) => this._visible.add(key));
    this.build();
  }

  hideAll() {
    this._visible.clear();
    this.build();
  }

  update(payload) {
    for (const [key, chart] of this._charts.entries()) {
      const def = chartDefs[key];
      def.series.forEach((series, index) => {
        chart.data.datasets[index].data = buildSeriesPoints(def, payload, series);
      });
      this._updateXAxis(chart, def, payload);
      this._updateAxisRanges(chart, key, def);
      chart.update('none');
    }
  }

  _destroyCharts() {
    for (const chart of this._charts.values()) {
      chart.destroy();
    }
    this._charts.clear();
  }

  _buildSidebar() {
    this._chartListElement.innerHTML = '';
    for (const [key, def] of Object.entries(chartDefs)) {
      const item = document.createElement('label');
      item.className = 'check-item';
      item.innerHTML = `<input type="checkbox" ${this._visible.has(key) ? 'checked' : ''} data-key="${key}"> <span>${def.title}</span>`;
      item.querySelector('input').addEventListener('change', (event) => {
        if (event.target.checked) {
          this._visible.add(key);
        } else {
          this._visible.delete(key);
        }
        this.build();
      });
      this._chartListElement.appendChild(item);
    }
  }

  _createChartCard(key, def) {
    const card = document.createElement('section');
    card.className = 'card';
    card.dataset.key = key;
    const axisHints = (def.axisHints || []).map((hint) => (
      `<span class="axis-hint"><span class="axis-swatch" style="background:${hint.color}"></span>${hint.text}</span>`
    )).join('');
    card.innerHTML = `<h3>${def.title}</h3>${axisHints ? `<div class="axis-hints">${axisHints}</div>` : ''}<canvas></canvas>`;
    const ctx = card.querySelector('canvas');
    const datasets = def.series.map((series) => ({
      label: series.label,
      data: [],
      parsing: false,
      borderColor: series.color,
      backgroundColor: series.color,
      borderWidth: 1.5,
      pointRadius: 0,
      tension: 0.1,
      yAxisID: series.axis || 'y',
      borderDash: series.dashed ? [6, 4] : [],
    }));
    const chart = new Chart(ctx, {
      type: 'line',
      data: { datasets },
      options: this._chartOptions(def),
    });
    this._charts.set(key, chart);
    return card;
  }

  _chartOptions(def) {
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
          display: def.series.some((series) => (series.axis || 'y') === 'y1'),
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

  _updateXAxis(chart, def, payload) {
    if (def.source === 'fft') {
      chart.options.scales.x.min = 0;
      chart.options.scales.x.max = Math.max(6, Number(payload.fft_meta?.x_max_hz || 12));
      return;
    }
    chart.options.scales.x.min = -Number(payload.window_sec || 10);
    chart.options.scales.x.max = 0;
  }

  _updateAxisRanges(chart, key, def) {
    ['y', 'y1'].forEach((axisId) => {
      const axisCfg = getAxisConfig(def, axisId);
      if (Number.isFinite(axisCfg.min) || Number.isFinite(axisCfg.max)) {
        if (Number.isFinite(axisCfg.min)) chart.options.scales[axisId].min = axisCfg.min;
        if (Number.isFinite(axisCfg.max)) chart.options.scales[axisId].max = axisCfg.max;
        return;
      }
      const values = chart.data.datasets
        .filter((dataset) => (dataset.yAxisID || 'y') === axisId)
        .flatMap((dataset) => dataset.data.map((point) => point.y));
      const target = computeAxisRange(values, axisCfg);
      if (!target) return;
      const blended = this._blendAxisRange(key, axisId, target, axisCfg);
      chart.options.scales[axisId].min = blended.min;
      chart.options.scales[axisId].max = blended.max;
    });
  }

  _blendAxisRange(key, axisId, target, cfg) {
    const stateKey = `${key}:${axisId}`;
    const current = this._axisState.get(stateKey);
    if (!current) {
      this._axisState.set(stateKey, target);
      return target;
    }
    const shrink = cfg.shrink ?? 0.16;
    const next = {
      min: target.min < current.min ? target.min : current.min + (target.min - current.min) * shrink,
      max: target.max > current.max ? target.max : current.max + (target.max - current.max) * shrink,
    };
    this._axisState.set(stateKey, next);
    return next;
  }
}

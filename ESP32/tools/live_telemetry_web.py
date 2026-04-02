#!/usr/bin/env python3
"""
Live web dashboard for ESP32 binary telemetry.

Features:
- asks the robot to start UDP binary telemetry toward this machine
- listens on UDP port 8888 by default
- serves a small web UI with a 10-second sliding window
- lets you show/hide multiple graphs at once
- uses only the Python standard library on the backend

Example:
  e:/AI/AutoBalancingAIBot/ESP32/.venv/Scripts/python.exe tools/live_telemetry_web.py \
      --robot-host 192.168.1.159
"""

from __future__ import annotations

import argparse
import cmath
import json
import math
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse


TELEMETRY_FMT_V6 = "<2I6f3f3ff2i8I6f4I6fI2f2I"
TELEMETRY_FMT_V5 = "<2I6f3f3ff2i8I6f4I6fI2f2I2i"
TELEMETRY_FMT_V3 = "<2I6f3f3ff2i8I6f4I6fI"
TELEMETRY_SIZE_V6 = struct.calcsize(TELEMETRY_FMT_V6)
TELEMETRY_SIZE_V5 = struct.calcsize(TELEMETRY_FMT_V5)
TELEMETRY_SIZE_V3 = struct.calcsize(TELEMETRY_FMT_V3)
PACKET_MAGIC = 0xABBA0001

HTML_PAGE = r"""
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ESP32 Live Telemetry</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.3/dist/chart.umd.min.js"></script>
  <style>
    :root {
      --bg: #0b1220;
      --panel: #121b2d;
      --panel-2: #182338;
      --text: #dbe7ff;
      --muted: #9ab0d6;
      --accent: #4ea1ff;
      --good: #24c37b;
      --warn: #f0b84b;
      --bad: #ff6b6b;
      --border: #24314d;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Inter, Segoe UI, Arial, sans-serif;
      background: var(--bg);
      color: var(--text);
    }
    .page {
      display: grid;
      grid-template-columns: 320px 1fr;
      min-height: 100vh;
    }
    .sidebar {
      padding: 18px;
      border-right: 1px solid var(--border);
      background: linear-gradient(180deg, var(--panel) 0%, #0f1728 100%);
    }
    .content {
      padding: 18px;
    }
    h1 {
      margin: 0 0 10px 0;
      font-size: 22px;
    }
    h2 {
      font-size: 14px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
      margin: 22px 0 10px;
    }
    .muted { color: var(--muted); }
    .status-pill {
      display: inline-block;
      padding: 4px 10px;
      border-radius: 999px;
      background: #17304d;
      color: #b8d9ff;
      font-size: 12px;
      margin-top: 8px;
    }
    .controls, .metrics {
      display: grid;
      gap: 10px;
    }
    .metric-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .metric {
      background: var(--panel-2);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 10px 12px;
    }
    .metric .label {
      color: var(--muted);
      font-size: 12px;
    }
    .metric .value {
      font-size: 20px;
      font-weight: 700;
      margin-top: 6px;
    }
    .check-list {
      display: grid;
      gap: 8px;
    }
    .check-item {
      display: flex;
      align-items: center;
      gap: 10px;
      background: var(--panel-2);
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 10px 12px;
    }
    .check-item input { accent-color: var(--accent); }
    .toolbar {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-bottom: 14px;
      align-items: center;
    }
    .toolbar button, .toolbar select {
      background: var(--panel);
      color: var(--text);
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 8px 12px;
      font: inherit;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(420px, 1fr));
      gap: 14px;
    }
    .card {
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 12px;
      min-height: 280px;
    }
    .card h3 {
      margin: 0 0 8px 0;
      font-size: 14px;
      color: #dbe7ff;
    }
    .axis-hints {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin-bottom: 8px;
    }
    .axis-hint {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 4px 8px;
      border-radius: 999px;
      border: 1px solid var(--border);
      background: var(--panel-2);
      color: var(--muted);
      font-size: 11px;
    }
    .axis-swatch {
      width: 10px;
      height: 10px;
      border-radius: 999px;
      display: inline-block;
    }
    .card canvas {
      width: 100% !important;
      height: 220px !important;
    }
    .footer-note {
      margin-top: 12px;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.5;
    }
    @media (max-width: 980px) {
      .page { grid-template-columns: 1fr; }
      .sidebar { border-right: none; border-bottom: 1px solid var(--border); }
    }
  </style>
</head>
<body>
  <div class="page">
    <aside class="sidebar">
      <h1>ESP32 Live Telemetry</h1>
      <div class="muted">Sliding window dashboard for binary UDP telemetry.</div>
      <div id="status" class="status-pill">Connecting…</div>

      <h2>Live metrics</h2>
      <div class="metrics metric-grid" id="metricGrid"></div>

      <h2>Charts</h2>
      <div class="check-list" id="chartList"></div>

      <div class="footer-note">
        The page keeps a rolling view of the last <span id="windowLabel">10</span> seconds.<br/>
        Graphs can be shown together to correlate timing, LQR terms, motor response and CPU.
      </div>
    </aside>

    <main class="content">
      <div class="toolbar">
        <button id="allBtn">Show all</button>
        <button id="noneBtn">Hide all</button>
        <label class="muted" for="refreshSel">Refresh</label>
        <select id="refreshSel">
          <option value="120">120 ms</option>
          <option value="200" selected>200 ms</option>
          <option value="333">333 ms</option>
          <option value="500">500 ms</option>
        </select>
      </div>
      <div class="grid" id="chartGrid"></div>
    </main>
  </div>

<script>
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
    ]
  },
  lqr: {
    title: 'LQR decomposition',
    axisHints: [
      { text: 'Left axis: LQR terms', color: '#4ea1ff' },
    ],
    scaleConfig: {
      y: { title: 'LQR terms', color: '#4ea1ff', pad: 0.14, shrink: 0.18 },
    },
    series: [
      { key: 'lqr_angle', label: 'Angle', color: '#4ea1ff' },
      { key: 'lqr_gyro', label: 'Gyro', color: '#f0b84b' },
      { key: 'lqr_dist', label: 'Distance', color: '#ff6b6b' },
      { key: 'lqr_speed', label: 'Speed', color: '#24c37b' },
    ]
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
    ]
  },
  cpu: {
    title: 'CPU load',
    axisHints: [
      { text: 'Left axis: CPU load %', color: '#4ea1ff' },
    ],
    scaleConfig: {
      y: { title: 'CPU %', color: '#4ea1ff', min: 0, max: 100 },
    },
    series: [
      { key: 'cpu0_pct', label: 'Core0 %', color: '#4ea1ff' },
      { key: 'cpu1_pct', label: 'Core1 %', color: '#ff6b6b' },
    ]
  },
  rpm: {
    title: 'Motor RPM (RS485)',
    axisHints: [
      { text: 'Left axis: motor RPM', color: '#4ea1ff' },
    ],
    scaleConfig: {
      y: { title: 'RPM', color: '#4ea1ff', pad: 0.10, shrink: 0.14 },
    },
    series: [
      { key: 'motor_rpm_l', label: 'Left RPM', color: '#4ea1ff' },
      { key: 'motor_rpm_r', label: 'Right RPM', color: '#ff6b6b' },
    ]
  },
  step: {
    title: 'Applied STEP frequency',
    axisHints: [
      { text: 'Left axis: applied STEP Hz', color: '#4ea1ff' },
    ],
    scaleConfig: {
      y: { title: 'STEP Hz', color: '#4ea1ff', pad: 0.10, shrink: 0.14 },
    },
    series: [
      { key: 'step_hz_l', label: 'Left STEP Hz', color: '#4ea1ff' },
      { key: 'step_hz_r', label: 'Right STEP Hz', color: '#ff6b6b' },
    ]
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
    ]
  },
  imu: {
    title: 'Raw IMU axes',
    axisHints: [
      { text: 'Left axis: raw IMU', color: '#4ea1ff' },
    ],
    scaleConfig: {
      y: { title: 'IMU raw', color: '#4ea1ff', pad: 0.14, shrink: 0.16 },
    },
    series: [
      { key: 'gx', label: 'Gyro X', color: '#4ea1ff' },
      { key: 'gy', label: 'Gyro Y', color: '#ff8f5a' },
      { key: 'gz', label: 'Gyro Z', color: '#9d7dff' },
      { key: 'ax', label: 'Accel X', color: '#24c37b' },
      { key: 'ay', label: 'Accel Y', color: '#f0b84b' },
      { key: 'az', label: 'Accel Z', color: '#dbe7ff' },
    ]
  },
  mixing: {
    title: 'Command mixing / saturation',
    axisHints: [
      { text: 'Left axis: commands / wheel mix', color: '#dbe7ff' },
    ],
    scaleConfig: {
      y: { title: 'Mix / command', color: '#dbe7ff', pad: 0.14, shrink: 0.16 },
    },
    series: [
      { key: 'cmd_raw', label: 'cmd raw', color: '#dbe7ff' },
      { key: 'cmd_final', label: 'cmd final', color: '#24c37b' },
      { key: 'left_preclip', label: 'left preclip', color: '#4ea1ff' },
      { key: 'right_preclip', label: 'right preclip', color: '#ff6b6b' },
      { key: 'left_postclip', label: 'left postclip', color: '#4ea1ff', dashed: true },
      { key: 'right_postclip', label: 'right postclip', color: '#ff6b6b', dashed: true },
    ]
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
    ]
  },
  fft: {
    title: 'Live FFT / oscillations',
    source: 'fft',
    axisHints: [
      { text: 'X axis: frequency (Hz)', color: '#4ea1ff' },
      { text: 'Y axis: relative magnitude', color: '#dbe7ff' },
    ],
    scaleConfig: {
      y: { title: 'Magnitude', color: '#dbe7ff', pad: 0.18, shrink: 0.18, quantileMin: 0.0, quantileMax: 1.0 },
    },
    series: [
      { key: 'pitch_mag', label: 'Pitch FFT', color: '#4ea1ff' },
      { key: 'cmd_mag', label: 'Command FFT', color: '#24c37b' },
      { key: 'rpm_mag', label: 'RPM FFT', color: '#ff8f5a' },
    ]
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
      }
    },
    plugins: {
      legend: { labels: { color: '#dbe7ff', boxWidth: 10 } },
      tooltip: { enabled: true }
    }
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
  return {
    min: min - span * pad,
    max: max + span * pad,
  };
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
  } catch (err) {
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
</script>
</body>
</html>
"""


@dataclass
class TelemetryPacket:
    timestamp_ms: int
    pitch_deg: float
    pid_in_deg: float
    pid_out: float
    iterm: float
    cmd: float
    steer: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    loop_freq_hz: float
    enc_l: int
    enc_r: int
    last_encoder_age_ms: int
    bus_latency_us: int
    ack_pending_left_us: int
    ack_pending_right_us: int
    bus_latency_left_us: int
    bus_latency_right_us: int
    bus_latency_left_age_ms: int
    bus_latency_right_age_ms: int
    lqr_angle: float
    lqr_gyro: float
    lqr_dist: float
    lqr_speed: float
    cpu0_pct: float
    cpu1_pct: float
    prof_f: int
    prof_l: int
    prof_t: int
    prof_log: int
    cmd_raw: float
    cmd_final: float
    left_preclip: float
    right_preclip: float
    left_postclip: float
    right_postclip: float
    sat_flags: int
    motor_rpm_l: float
    motor_rpm_r: float
    step_hz_l: int
    step_hz_r: int

    def to_dict(self, latest_ts_ms: int) -> dict[str, Any]:
        return {
            **self.__dict__,
            "t_s": (self.timestamp_ms - latest_ts_ms) / 1000.0,
        }


def _resample_points(samples: list[dict[str, Any]], target: int = 128) -> list[dict[str, Any]]:
    if len(samples) <= target:
        return samples
    if target <= 1:
        return [samples[-1]]
    out: list[dict[str, Any]] = []
    last_index = len(samples) - 1
    for i in range(target):
        idx = round(i * last_index / (target - 1))
        out.append(samples[idx])
    return out


def _detrend(values: list[float]) -> list[float]:
    n = len(values)
    if n <= 2:
        mean = sum(values) / max(1, n)
        return [v - mean for v in values]
    start = values[0]
    end = values[-1]
    slope = (end - start) / (n - 1)
    return [values[i] - (start + slope * i) for i in range(n)]


def _compute_fft(samples: list[dict[str, Any]], max_freq_hz: float = 25.0) -> list[dict[str, float]]:
    points = _resample_points(samples, target=128)
    if len(points) < 32:
        return []

    dt_values = [
        (points[i]["timestamp_ms"] - points[i - 1]["timestamp_ms"]) / 1000.0
        for i in range(1, len(points))
        if points[i]["timestamp_ms"] > points[i - 1]["timestamp_ms"]
    ]
    if not dt_values:
        return []

    dt_avg = sum(dt_values) / len(dt_values)
    if dt_avg <= 0.0:
        return []

    fs = 1.0 / dt_avg
    n = len(points)
    max_k = min(n // 2, int(max_freq_hz * n / fs))
    if max_k <= 1:
        return []

    window = [0.5 - 0.5 * math.cos((2.0 * math.pi * i) / (n - 1)) for i in range(n)]
    pitch = [points[i]["pitch_deg"] for i in range(n)]
    cmd = [points[i]["cmd_final"] for i in range(n)]
    rpm = [points[i]["rpm_avg"] for i in range(n)]
    pitch_w = [v * w for v, w in zip(_detrend(pitch), window)]
    cmd_w = [v * w for v, w in zip(_detrend(cmd), window)]
    rpm_w = [v * w for v, w in zip(_detrend(rpm), window)]

    rows: list[dict[str, float]] = []
    for k in range(1, max_k + 1):
        freq_hz = (k * fs) / n
        if freq_hz > max_freq_hz:
            break
        rot = [cmath.exp((-2j * math.pi * k * i) / n) for i in range(n)]
        pitch_sum = sum(pitch_w[i] * rot[i] for i in range(n))
        cmd_sum = sum(cmd_w[i] * rot[i] for i in range(n))
        rpm_sum = sum(rpm_w[i] * rot[i] for i in range(n))
        rows.append(
            {
                "freq_hz": freq_hz,
                "pitch_mag": abs(pitch_sum) / n,
                "cmd_mag": abs(cmd_sum) / n,
                "rpm_mag": abs(rpm_sum) / n,
            }
        )
    return rows


def _enrich_samples(samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not samples:
        return samples

    first_avg = (samples[0]["enc_l"] + samples[0]["enc_r"]) / 2.0
    first_diff = (samples[0]["enc_l"] - samples[0]["enc_r"]) / 2.0
    prev = None
    loop_ema = float(samples[0].get("loop_freq_hz", 0.0) or 0.0)

    for sample in samples:
        avg_enc = (sample["enc_l"] + sample["enc_r"]) / 2.0
        diff_enc = (sample["enc_l"] - sample["enc_r"]) / 2.0
        sample["enc_avg_rel"] = avg_enc - first_avg
        sample["enc_diff_rel"] = diff_enc - first_diff
        sample["rpm_avg"] = (sample["motor_rpm_l"] + sample["motor_rpm_r"]) / 2.0
        sample["rpm_diff"] = sample["motor_rpm_l"] - sample["motor_rpm_r"]
        sample["step_avg"] = (sample["step_hz_l"] + sample["step_hz_r"]) / 2.0
        sample["step_diff"] = sample["step_hz_l"] - sample["step_hz_r"]

        loop_hz = float(sample.get("loop_freq_hz", 0.0) or 0.0)
        loop_ema = (0.85 * loop_ema) + (0.15 * loop_hz)
        sample["loop_freq_hz_ema"] = loop_ema

        motion_active = (
            max(abs(sample["step_hz_l"]), abs(sample["step_hz_r"])) > 50
            or max(abs(sample["motor_rpm_l"]), abs(sample["motor_rpm_r"])) > 8.0
            or max(abs(sample["cmd_final"]), abs(sample["left_postclip"]), abs(sample["right_postclip"])) > 0.03
        )
        sample["motion_active"] = motion_active
        sample["encoder_age_plot_ms"] = (
            min(float(sample["last_encoder_age_ms"]), 300.0)
            if motion_active and sample["last_encoder_age_ms"] < 5000
            else None
        )

        if prev is None:
            sample["enc_vel_proxy"] = 0.0
            sample["pitch_rate_est_dps"] = 0.0
        else:
            dt_s = max(1e-3, (sample["timestamp_ms"] - prev["timestamp_ms"]) / 1000.0)
            sample["enc_vel_proxy"] = (avg_enc - prev["_avg_enc"]) / dt_s
            sample["pitch_rate_est_dps"] = (sample["pitch_deg"] - prev["pitch_deg"]) / dt_s

        sample["_avg_enc"] = avg_enc
        prev = sample

    for sample in samples:
        sample.pop("_avg_enc", None)
    return samples


class DashboardState:
    def __init__(self, window_sec: float) -> None:
        self.window_sec = window_sec
        self.samples: deque[TelemetryPacket] = deque()
        self.lock = threading.Lock()
        self.packet_count = 0
        self.last_rx_monotonic = 0.0
        self.status = "Waiting for telemetry…"

    def add(self, packet: TelemetryPacket) -> None:
        with self.lock:
            self.samples.append(packet)
            self.packet_count += 1
            self.last_rx_monotonic = time.monotonic()
            self.status = "Receiving telemetry"
            cutoff_ms = packet.timestamp_ms - int(self.window_sec * 1000.0)
            while self.samples and self.samples[0].timestamp_ms < cutoff_ms:
                self.samples.popleft()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            samples = list(self.samples)
            latest = samples[-1] if samples else None
            if latest is None:
                status = self.status
                if self.last_rx_monotonic and (time.monotonic() - self.last_rx_monotonic) > 2.0:
                    status = "Telemetry timeout"
                return {
                    "window_sec": self.window_sec,
                    "samples": [],
                    "latest": None,
                    "status": status,
                }

            status = self.status
            if self.last_rx_monotonic and (time.monotonic() - self.last_rx_monotonic) > 2.0:
                status = "Telemetry timeout"

            latest_ts = latest.timestamp_ms
            decimate_target = 900
            stride = max(1, len(samples) // decimate_target)
            sliced = samples[::stride]
            if sliced[-1] is not latest:
                sliced.append(latest)
            sample_dicts = [pkt.to_dict(latest_ts) for pkt in sliced]
            sample_dicts = _enrich_samples(sample_dicts)
            latest_dict = sample_dicts[-1].copy()
            latest_dict["packet_count"] = self.packet_count
            return {
                "window_sec": self.window_sec,
                "samples": sample_dicts,
                "fft": _compute_fft(sample_dicts),
                "latest": latest_dict,
                "status": status,
            }


def parse_packet(data: bytes) -> TelemetryPacket | None:
    if len(data) == TELEMETRY_SIZE_V6:
        values = struct.unpack(TELEMETRY_FMT_V6, data)
        if values[0] != PACKET_MAGIC:
            return None
        return TelemetryPacket(
            timestamp_ms=values[1],
            pitch_deg=values[2],
            pid_in_deg=values[3],
            pid_out=values[4],
            iterm=values[5],
            cmd=values[6],
            steer=values[7],
            ax=values[8], ay=values[9], az=values[10],
            gx=values[11], gy=values[12], gz=values[13],
            loop_freq_hz=values[14],
            enc_l=values[15], enc_r=values[16],
            last_encoder_age_ms=values[17],
            bus_latency_us=values[18],
            ack_pending_left_us=values[19],
            ack_pending_right_us=values[20],
            bus_latency_left_us=values[21],
            bus_latency_right_us=values[22],
            bus_latency_left_age_ms=values[23],
            bus_latency_right_age_ms=values[24],
            lqr_angle=values[25],
            lqr_gyro=values[26],
            lqr_dist=values[27],
            lqr_speed=values[28],
            cpu0_pct=values[29],
            cpu1_pct=values[30],
            prof_f=values[31],
            prof_l=values[32],
            prof_t=values[33],
            prof_log=values[34],
            cmd_raw=values[35],
            cmd_final=values[36],
            left_preclip=values[37],
            right_preclip=values[38],
            left_postclip=values[39],
            right_postclip=values[40],
            sat_flags=values[41],
            motor_rpm_l=values[42],
            motor_rpm_r=values[43],
            step_hz_l=values[44],
            step_hz_r=values[45],
        )

    if len(data) == TELEMETRY_SIZE_V5:
        values = struct.unpack(TELEMETRY_FMT_V5, data)
        if values[0] != PACKET_MAGIC:
            return None
        return TelemetryPacket(
            timestamp_ms=values[1],
            pitch_deg=values[2],
            pid_in_deg=values[3],
            pid_out=values[4],
            iterm=values[5],
            cmd=values[6],
            steer=values[7],
            ax=values[8], ay=values[9], az=values[10],
            gx=values[11], gy=values[12], gz=values[13],
            loop_freq_hz=values[14],
            enc_l=values[15], enc_r=values[16],
            last_encoder_age_ms=values[17],
            bus_latency_us=values[18],
            ack_pending_left_us=values[19],
            ack_pending_right_us=values[20],
            bus_latency_left_us=values[21],
            bus_latency_right_us=values[22],
            bus_latency_left_age_ms=values[23],
            bus_latency_right_age_ms=values[24],
            lqr_angle=values[25],
            lqr_gyro=values[26],
            lqr_dist=values[27],
            lqr_speed=values[28],
            cpu0_pct=values[29],
            cpu1_pct=values[30],
            prof_f=values[31],
            prof_l=values[32],
            prof_t=values[33],
            prof_log=values[34],
            cmd_raw=values[35],
            cmd_final=values[36],
            left_preclip=values[37],
            right_preclip=values[38],
            left_postclip=values[39],
            right_postclip=values[40],
            sat_flags=values[41],
            motor_rpm_l=values[42],
            motor_rpm_r=values[43],
            step_hz_l=values[44],
            step_hz_r=values[45],
        )

    if len(data) == TELEMETRY_SIZE_V3:
        values = struct.unpack(TELEMETRY_FMT_V3, data)
        if values[0] != PACKET_MAGIC:
            return None
        return TelemetryPacket(
            timestamp_ms=values[1],
            pitch_deg=values[2],
            pid_in_deg=values[3],
            pid_out=values[4],
            iterm=values[5],
            cmd=values[6],
            steer=values[7],
            ax=values[8], ay=values[9], az=values[10],
            gx=values[11], gy=values[12], gz=values[13],
            loop_freq_hz=values[14],
            enc_l=values[15], enc_r=values[16],
            last_encoder_age_ms=values[17],
            bus_latency_us=values[18],
            ack_pending_left_us=values[19],
            ack_pending_right_us=values[20],
            bus_latency_left_us=values[21],
            bus_latency_right_us=values[22],
            bus_latency_left_age_ms=values[23],
            bus_latency_right_age_ms=values[24],
            lqr_angle=values[25],
            lqr_gyro=values[26],
            lqr_dist=values[27],
            lqr_speed=values[28],
            cpu0_pct=values[29],
            cpu1_pct=values[30],
            prof_f=values[31],
            prof_l=values[32],
            prof_t=values[33],
            prof_log=values[34],
            cmd_raw=values[35],
            cmd_final=values[36],
            left_preclip=values[37],
            right_preclip=values[38],
            left_postclip=values[39],
            right_postclip=values[40],
            sat_flags=values[41],
            motor_rpm_l=0.0,
            motor_rpm_r=0.0,
            step_hz_l=0,
            step_hz_r=0,
        )

    return None


class TelemetryReceiver(threading.Thread):
    def __init__(self, state: DashboardState, bind_host: str, udp_port: int) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.bind_host = bind_host
        self.udp_port = udp_port
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_host, self.udp_port))
        sock.settimeout(0.5)
        self.state.status = f"Listening on UDP {self.bind_host or '0.0.0.0'}:{self.udp_port}"
        while not self._stop.is_set():
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            packet = parse_packet(data)
            if packet is not None:
                self.state.add(packet)
        sock.close()


class RobotConsoleClient:
    def __init__(self, host: str, port: int, timeout: float = 5.0) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout

    def send(self, *commands: str) -> list[str]:
        lines: list[str] = []
        with socket.create_connection((self.host, self.port), timeout=self.timeout) as sock:
            sock.setblocking(False)
            time.sleep(0.2)
            self._drain(sock, lines, 0.3)
            for command in commands:
                if not command.endswith("\n"):
                    command += "\n"
                sock.sendall(command.encode("utf-8"))
                self._drain(sock, lines, 0.35)
        return lines

    @staticmethod
    def _drain(sock: socket.socket, lines: list[str], quiet_s: float) -> None:
        deadline = time.monotonic() + quiet_s
        while time.monotonic() < deadline:
            try:
                chunk = sock.recv(8192)
            except BlockingIOError:
                time.sleep(0.02)
                continue
            if not chunk:
                break
            text = chunk.decode("utf-8", errors="ignore")
            for line in text.splitlines():
                if line.strip():
                    lines.append(line.rstrip())
            deadline = time.monotonic() + quiet_s


class DashboardHttpServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], state: DashboardState, console: RobotConsoleClient | None):
        super().__init__(server_address, DashboardHandler)
        self.state = state
        self.console = console


class DashboardHandler(BaseHTTPRequestHandler):
    server: DashboardHttpServer

    def _json(self, payload: dict[str, Any], status: int = 200) -> None:
        raw = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            raw = HTML_PAGE.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
            return
        if parsed.path == "/api/state":
            self._json(self.server.state.snapshot())
            return
        if parsed.path == "/api/command":
            cmd = parse_qs(parsed.query).get("cmd", [""])[0].strip()
            if not cmd:
                self._json({"ok": False, "error": "Missing cmd"}, status=400)
                return
            if not self.server.console:
                self._json({"ok": False, "error": "Console disabled"}, status=400)
                return
            try:
                lines = self.server.console.send(cmd)
                self._json({"ok": True, "lines": lines})
            except OSError as exc:
                self._json({"ok": False, "error": str(exc)}, status=502)
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, format: str, *args: Any) -> None:
        return


def main() -> int:
    parser = argparse.ArgumentParser(description="Live web dashboard for ESP32 binary telemetry")
    parser.add_argument("--robot-host", required=True, help="ESP32 Wi-Fi console IP")
    parser.add_argument("--robot-port", type=int, default=2333, help="ESP32 Wi-Fi console TCP port")
    parser.add_argument("--udp-port", type=int, default=8888, help="Local UDP port for binary telemetry")
    parser.add_argument("--http-host", default="0.0.0.0", help="HTTP bind host")
    parser.add_argument("--http-port", type=int, default=8000, help="HTTP bind port")
    parser.add_argument("--window-sec", type=float, default=10.0, help="Sliding visualization window in seconds")
    parser.add_argument("--no-auto-start", action="store_true", help="Do not send SYS TELEM UDP AUTO to the robot")
    args = parser.parse_args()

    state = DashboardState(window_sec=args.window_sec)
    console = RobotConsoleClient(args.robot_host, args.robot_port)

    if not args.no_auto_start:
        try:
            lines = console.send("SYS TELEM UDP STOP", "SYS TELEM UDP AUTO")
            if lines:
                print("Robot console:")
                for line in lines:
                    print(f"  {line}")
        except OSError as exc:
            print(f"Warning: could not arm UDP telemetry automatically: {exc}")

    bind_host = "" if args.http_host == "0.0.0.0" else args.http_host
    receiver = TelemetryReceiver(state=state, bind_host=bind_host, udp_port=args.udp_port)
    receiver.start()

    server = DashboardHttpServer((args.http_host, args.http_port), state=state, console=console)
    print(f"Dashboard listening on http://127.0.0.1:{args.http_port}")
    print(f"Robot console target: {args.robot_host}:{args.robot_port}")
    print(f"UDP telemetry port: {args.udp_port} | window: {args.window_sec:.1f}s")

    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\nStopping dashboard…")
    finally:
        receiver.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

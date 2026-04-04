const CAPTURE_CHART_SPECS = [
  {
    key: "pitch",
    title: "Pitch / command",
    xMax: 0,
    datasets: [
      { key: "pitch_deg", label: "Pitch (deg)", color: "#4ea1ff", axis: "y" },
      { key: "pid_in_deg", label: "PID input (deg)", color: "#9d7dff", axis: "y" },
      { key: "cmd_final", label: "cmd final", color: "#24c37b", axis: "y1" },
      { key: "steer", label: "steer", color: "#ff8f5a", axis: "y1" },
    ],
  },
  {
    key: "lqr",
    title: "LQR decomposition",
    xMax: 0,
    datasets: [
      { key: "lqr_angle", label: "Angle", color: "#4ea1ff", axis: "y" },
      { key: "lqr_gyro", label: "Gyro", color: "#f0b84b", axis: "y" },
      { key: "lqr_dist", label: "Distance", color: "#ff6b6b", axis: "y" },
      { key: "lqr_speed", label: "Speed", color: "#24c37b", axis: "y" },
    ],
  },
  {
    key: "drive",
    title: "Motor drive / RPM",
    xMax: 0,
    datasets: [
      { key: "left_postclip", label: "Left postclip", color: "#4ea1ff", axis: "y" },
      { key: "right_postclip", label: "Right postclip", color: "#ff6b6b", axis: "y" },
      { key: "motor_rpm_l", label: "Left RPM", color: "#24c37b", axis: "y1" },
      { key: "motor_rpm_r", label: "Right RPM", color: "#f0b84b", axis: "y1" },
    ],
  },
  {
    key: "timing",
    title: "Timing / encoder health",
    xMax: 0,
    datasets: [
      { key: "loop_freq_hz_ema", label: "Loop Hz (EMA)", color: "#ff6b6b", axis: "y" },
      { key: "prof_t", label: "Total us", color: "#dbe7ff", axis: "y1" },
      { key: "encoder_age_plot_ms", label: "Encoder age ms", color: "#24c37b", axis: "y1" },
    ],
  },
];

function metric(label, value) {
  return `<div class="metric"><div class="label">${label}</div><div class="value">${value}</div></div>`;
}

function formatUnixMs(unixMs) {
  if (!Number.isFinite(unixMs) || unixMs <= 0) return '—';
  return new Date(unixMs).toLocaleString();
}

function formatNumber(value, digits = 2, suffix = '') {
  if (!Number.isFinite(value)) return '—';
  return `${Number(value).toFixed(digits)}${suffix}`;
}

function samplePoints(samples, key) {
  return (samples || [])
    .map((sample) => ({ x: sample.t_s, y: sample[key] }))
    .filter((point) => Number.isFinite(point.y));
}

function computeRange(points, padRatio = 0.14) {
  const values = points.map((point) => point.y).filter((value) => Number.isFinite(value));
  if (!values.length) return null;
  let min = Math.min(...values);
  let max = Math.max(...values);
  if (min === max) {
    const widen = Math.max(1, Math.abs(min) * 0.1);
    min -= widen;
    max += widen;
  }
  const span = Math.max(1e-6, max - min);
  return { min: min - span * padRatio, max: max + span * padRatio };
}

export class CapturePanel {
  constructor(documentRef, api) {
    this._document = documentRef;
    this._window = this._document.defaultView || window;
    this._api = api;
    this._statusElement = this._document.getElementById('captureStatus');
    this._storageElement = this._document.getElementById('captureStoragePath');
    this._labelInput = this._document.getElementById('captureLabel');
    this._autoModeInput = this._document.getElementById('captureAutoMode');
    this._armButton = this._document.getElementById('captureArmBtn');
    this._stopButton = this._document.getElementById('captureStopBtn');
    this._refreshButton = this._document.getElementById('captureRefreshBtn');
    this._summaryElement = this._document.getElementById('captureSummary');
    this._sourceElement = this._document.getElementById('captureSource');
    this._runsElement = this._document.getElementById('captureRunsList');
    this._deleteSelectedButton = this._document.getElementById('captureDeleteSelectedBtn');
    this._selectionSummaryElement = this._document.getElementById('captureSelectionSummary');
    this._charts = new Map();
    this._pollHandle = null;
    this._selectedCaptureId = null;
    this._selectedPayload = null;
    this._latestStatusPayload = null;
    this._batchSelection = new Set();
    this._refreshInFlight = false;
    this._actionInFlight = false;
  }

  start() {
    this._buildCharts();
    this._wireControls();
    void this.refresh();
  }

  activate() {
    if (this._pollHandle) return;
    this._pollHandle = setInterval(() => {
      void this.refresh();
    }, 600);
    void this.refresh();
  }

  deactivate() {
    if (!this._pollHandle) return;
    clearInterval(this._pollHandle);
    this._pollHandle = null;
  }

  async refresh() {
    if (this._refreshInFlight) {
      return;
    }
    this._refreshInFlight = true;
    try {
      const statusPayload = await this._api.fetchCaptureStatus();
      this._latestStatusPayload = statusPayload;
      this._renderStatus(statusPayload);
      const recentCaptures = statusPayload.recent_captures || [];
      this._pruneBatchSelection(recentCaptures);
      this._renderRecentCaptures(recentCaptures);

      const currentCapture = statusPayload.current_capture;
      if (currentCapture && (!this._selectedCaptureId || this._selectedCaptureId === currentCapture.capture_id)) {
        this._selectedCaptureId = currentCapture.capture_id;
        this._selectedPayload = currentCapture;
      }
      if (!this._selectedPayload && currentCapture) {
        this._selectedPayload = currentCapture;
      }
      if (!currentCapture && statusPayload.state === 'idle' && !recentCaptures.some((capture) => capture.capture_id === this._selectedCaptureId)) {
        this._selectedCaptureId = null;
        this._selectedPayload = null;
      }
      this._renderSelectedCapture();
    } catch (error) {
      this._setStatus(error.message || 'Capture API unavailable', true);
    } finally {
      this._refreshInFlight = false;
    }
  }

  async loadCapture(captureId) {
    try {
      this._selectedPayload = await this._api.fetchCaptureRun(captureId);
      this._selectedCaptureId = captureId;
      this._renderSelectedCapture();
    } catch (error) {
      this._setStatus(error.message || 'Unable to load capture', true);
    }
  }

  async deleteCapture(captureId) {
    if (!this._window.confirm(`Delete capture ${captureId}?`)) {
      return;
    }
    try {
      const statusPayload = await this._api.deleteCapture(captureId);
      this._batchSelection.delete(captureId);
      if (this._selectedCaptureId === captureId) {
        this._selectedCaptureId = null;
        this._selectedPayload = null;
      }
      this._latestStatusPayload = statusPayload;
      this._renderStatus(statusPayload);
      this._renderRecentCaptures(statusPayload.recent_captures || []);
      this._renderSelectedCapture();
    } catch (error) {
      this._setStatus(error.message || 'Unable to delete capture', true);
    }
  }

  async deleteSelectedCaptures() {
    const captureIds = [...this._batchSelection];
    if (!captureIds.length) {
      return;
    }
    const label = captureIds.length === 1 ? 'this capture' : `${captureIds.length} captures`;
    if (!this._window.confirm(`Delete ${label}?`)) {
      return;
    }
    this._actionInFlight = true;
    this._deleteSelectedButton.disabled = true;
    try {
      for (const captureId of captureIds) {
        const statusPayload = await this._api.deleteCapture(captureId);
        this._latestStatusPayload = statusPayload;
        this._batchSelection.delete(captureId);
        if (this._selectedCaptureId === captureId) {
          this._selectedCaptureId = null;
          this._selectedPayload = null;
        }
      }
      await this.refresh();
    } catch (error) {
      this._setStatus(error.message || 'Unable to delete selected captures', true);
    } finally {
      this._actionInFlight = false;
      if (this._latestStatusPayload) {
        this._renderStatus(this._latestStatusPayload);
      }
      this._updateBatchSelectionUi();
    }
  }

  _wireControls() {
    this._armButton.addEventListener('click', async () => {
      if (this._actionInFlight) {
        return;
      }
      this._actionInFlight = true;
      this._armButton.disabled = true;
      this._stopButton.disabled = true;
      try {
        const captureMode = this._autoModeInput?.checked ? 'auto' : 'manual';
        const result = await this._api.armCapture(this._labelInput.value.trim(), captureMode);
        this._selectedCaptureId = result.current_capture?.capture_id || null;
        this._selectedPayload = result.current_capture || null;
        await this.refresh();
      } catch (error) {
        this._setStatus(error.message || 'Unable to arm capture', true);
      } finally {
        this._actionInFlight = false;
        if (this._latestStatusPayload) {
          this._renderStatus(this._latestStatusPayload);
        }
      }
    });

    this._stopButton.addEventListener('click', async () => {
      if (this._actionInFlight) {
        return;
      }
      this._actionInFlight = true;
      this._armButton.disabled = true;
      this._stopButton.disabled = true;
      try {
        const result = await this._api.stopCapture();
        this._selectedCaptureId = result.current_capture?.capture_id || null;
        this._selectedPayload = result.current_capture || null;
        await this.refresh();
      } catch (error) {
        this._setStatus(error.message || 'Unable to stop capture', true);
      } finally {
        this._actionInFlight = false;
        if (this._latestStatusPayload) {
          this._renderStatus(this._latestStatusPayload);
        }
      }
    });

    this._refreshButton.addEventListener('click', () => {
      void this.refresh();
    });

    this._deleteSelectedButton.addEventListener('click', () => {
      if (this._actionInFlight) {
        return;
      }
      void this.deleteSelectedCaptures();
    });

    this._runsElement.addEventListener('click', (event) => {
      const loadButton = event.target.closest('[data-load-capture]');
      if (loadButton) {
        void this.loadCapture(loadButton.dataset.loadCapture);
        return;
      }
      const deleteButton = event.target.closest('[data-delete-capture]');
      if (deleteButton) {
        void this.deleteCapture(deleteButton.dataset.deleteCapture);
      }
    });

    this._runsElement.addEventListener('change', (event) => {
      const checkbox = event.target.closest('[data-select-capture]');
      if (!checkbox) {
        return;
      }
      const captureId = checkbox.dataset.selectCapture;
      if (!captureId) {
        return;
      }
      if (checkbox.checked) {
        this._batchSelection.add(captureId);
      } else {
        this._batchSelection.delete(captureId);
      }
      this._updateBatchSelectionUi();
    });
  }

  _renderStatus(statusPayload) {
    const state = statusPayload.state || 'idle';
    const stateLabel = {
      idle: 'Idle',
      armed: 'Armed — waiting for capture trigger',
      capturing: 'Capturing live telemetry',
      completed: 'Last capture completed',
    }[state] || state;
    this._setStatus(stateLabel, false);
    this._storageElement.textContent = statusPayload.capture_root || 'captures/live_telemetry';
    this._armButton.disabled = state === 'armed' || state === 'capturing';
    this._stopButton.disabled = state === 'idle';
    this._deleteSelectedButton.disabled = this._batchSelection.size === 0 || this._actionInFlight;
  }

  _renderSelectedCapture() {
    const capturePayload = this._selectedPayload;
    if (!capturePayload) {
      this._sourceElement.textContent = 'No capture selected yet.';
      this._summaryElement.innerHTML = metric('State', 'Idle') + metric('Packets', '0') + metric('Duration', '—') + metric('Dominant pitch FFT', '—');
      this._updateCharts({ samples: [], fft: [], fft_meta: { x_max_hz: 25, sample_rate_hz: null } });
      return;
    }

    const summary = capturePayload.summary || {};
    const preview = capturePayload.preview || { samples: [], fft: [], fft_meta: { x_max_hz: 25, sample_rate_hz: null } };
    this._sourceElement.textContent = `${capturePayload.label} · mode ${capturePayload.mode || 'manual'} · started ${formatUnixMs(capturePayload.started_at_unix_ms)} · stop ${capturePayload.stop_reason || '—'}`;
    this._summaryElement.innerHTML = [
      metric('State', capturePayload.status || '—'),
      metric('Mode', capturePayload.mode || 'manual'),
      metric('Packets', capturePayload.packet_count ?? '—'),
      metric('Duration', formatNumber(summary.duration_s, 2, ' s')),
      metric('Max |pitch|', formatNumber(summary.max_abs_pitch_deg, 2, '°')),
      metric('Max |cmd|', formatNumber(summary.max_abs_cmd_final, 3)),
      metric('Loop avg', formatNumber(summary.average_loop_hz, 1, ' Hz')),
      metric('Sat ratio', formatNumber((summary.saturation_ratio ?? 0) * 100, 1, '%')),
      metric('Dominant pitch FFT', formatNumber(summary.dominant_pitch_fft_hz, 2, ' Hz')),
    ].join('');
    this._updateCharts(preview);
  }

  _renderRecentCaptures(captures) {
    if (!captures.length) {
      this._runsElement.innerHTML = '<div class="muted">No saved captures yet.</div>';
      this._updateBatchSelectionUi();
      return;
    }
    this._runsElement.innerHTML = captures.map((capture) => `
      <article class="capture-run-card ${capture.capture_id === this._selectedCaptureId ? 'is-selected' : ''}">
        <div class="capture-run-header">
          <div>
            <h3>${capture.label}</h3>
            <div class="muted">${formatUnixMs(capture.started_at_unix_ms)} · mode ${capture.mode || 'manual'} · ${capture.packet_count || 0} packets · ${capture.stop_reason || 'manual_stop'}</div>
          </div>
          <div class="button-row">
            <button type="button" data-load-capture="${capture.capture_id}">Load</button>
            <button type="button" class="button-danger" data-delete-capture="${capture.capture_id}">Delete</button>
            <label class="capture-select-toggle" title="Select for batch delete">
              <input type="checkbox" data-select-capture="${capture.capture_id}" ${this._batchSelection.has(capture.capture_id) ? 'checked' : ''} />
            </label>
          </div>
        </div>
        <div class="capture-run-links">
          <a href="${capture.files.csv}" download>CSV</a>
          <a href="${capture.files.json}" download>JSON</a>
          <a href="${capture.files.log}" download>LOG</a>
        </div>
      </article>
    `).join('');
    this._updateBatchSelectionUi();
  }

  _pruneBatchSelection(captures) {
    const validCaptureIds = new Set((captures || []).map((capture) => capture.capture_id));
    for (const captureId of [...this._batchSelection]) {
      if (!validCaptureIds.has(captureId)) {
        this._batchSelection.delete(captureId);
      }
    }
  }

  _updateBatchSelectionUi() {
    const selectionCount = this._batchSelection.size;
    this._selectionSummaryElement.textContent = `${selectionCount} selected`;
    this._deleteSelectedButton.disabled = selectionCount === 0 || this._actionInFlight;
  }

  _buildCharts() {
    for (const spec of CAPTURE_CHART_SPECS) {
      const canvas = this._document.getElementById(`captureChart-${spec.key}`);
      const datasets = spec.datasets.map((dataset) => ({
        label: dataset.label,
        data: [],
        parsing: false,
        borderColor: dataset.color,
        backgroundColor: dataset.color,
        borderWidth: 1.5,
        pointRadius: 0,
        tension: 0.12,
        yAxisID: dataset.axis || 'y',
      }));
      const chart = new Chart(canvas, {
        type: 'line',
        data: { datasets },
        options: {
          responsive: true,
          animation: false,
          maintainAspectRatio: false,
          interaction: { mode: 'nearest', intersect: false },
          scales: {
            x: {
              type: 'linear',
              min: -10,
              max: spec.xMax,
              ticks: { color: '#9ab0d6' },
              grid: { color: 'rgba(154,176,214,0.12)' },
            },
            y: {
              ticks: { color: '#9ab0d6' },
              grid: { color: 'rgba(154,176,214,0.12)' },
            },
            y1: {
              position: 'right',
              display: spec.datasets.some((dataset) => dataset.axis === 'y1'),
              ticks: { color: '#9ab0d6' },
              grid: { drawOnChartArea: false },
            },
          },
          plugins: {
            legend: { labels: { color: '#dbe7ff', boxWidth: 10 } },
          },
        },
      });
      this._charts.set(spec.key, { chart, spec });
    }
  }

  _updateCharts(previewPayload) {
    const samples = previewPayload.samples || [];
    const xMin = samples.length ? Math.min(...samples.map((sample) => sample.t_s)) : -10;
    for (const { chart, spec } of this._charts.values()) {
      spec.datasets.forEach((dataset, index) => {
        chart.data.datasets[index].data = samplePoints(samples, dataset.key);
      });
      chart.options.scales.x.min = Math.min(-0.5, xMin);
      chart.options.scales.x.max = 0;
      this._applyAxisRanges(chart);
      chart.update('none');
    }
  }

  _applyAxisRanges(chart) {
    ['y', 'y1'].forEach((axisId) => {
      const points = chart.data.datasets
        .filter((dataset) => (dataset.yAxisID || 'y') === axisId)
        .flatMap((dataset) => dataset.data);
      const range = computeRange(points);
      if (!range) return;
      chart.options.scales[axisId].min = range.min;
      chart.options.scales[axisId].max = range.max;
    });
  }

  _setStatus(message, isError) {
    this._statusElement.textContent = message;
    this._statusElement.classList.toggle('is-error', Boolean(isError));
  }
}
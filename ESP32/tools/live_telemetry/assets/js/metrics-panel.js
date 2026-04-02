function metric(name, value) {
  return `<div class="metric"><div class="label">${name}</div><div class="value">${value}</div></div>`;
}

export class MetricsPanel {
  constructor(statusElement, metricGridElement) {
    this._statusElement = statusElement;
    this._metricGridElement = metricGridElement;
  }

  render(latest, statusText) {
    if (!latest) {
      this._metricGridElement.innerHTML = metric('Packets', '0') + metric('Loop Hz', '—') + metric('Pitch', '—') + metric('CPU1', '—');
      this._statusElement.textContent = statusText || 'Waiting for telemetry…';
      return;
    }

    this._metricGridElement.innerHTML = [
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
    this._statusElement.textContent = statusText;
  }

  setDisconnected() {
    this._statusElement.textContent = 'Dashboard disconnected';
  }
}

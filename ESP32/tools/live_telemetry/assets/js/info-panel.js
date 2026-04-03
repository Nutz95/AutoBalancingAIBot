function infoRow(label, value) {
  return `<div class="info-row"><span class="info-label">${label}</span><span class="info-value">${value}</span></div>`;
}

function asFixed(value, digits = 3) {
  return Number(value ?? 0).toFixed(digits);
}

export class InfoPanel {
  constructor(documentRef, api) {
    this._document = documentRef;
    this._api = api;
    this._gridElement = this._document.getElementById('infoGrid');
    this._refreshButton = this._document.getElementById('infoRefreshBtn');
    this._statusElement = this._document.getElementById('infoStatus');
  }

  start() {
    this._refreshButton.addEventListener('click', () => {
      void this.load(true);
    });
  }

  async load(forceRefresh = false) {
    this._statusElement.textContent = 'Refreshing info…';
    try {
      const config = forceRefresh ? await this._api.refreshConfig() : await this._api.fetchConfig();
      this.render(config);
      this._statusElement.textContent = 'Info synced';
    } catch (error) {
      this._statusElement.textContent = error.message || 'Failed to load info';
    }
  }

  render(config) {
    const controller = config.controller || {};
    const lqr = controller.lqr || {};
    const motors = config.motors || {};
    const system = config.system || {};
    const logs = config.logs || {};
    const filter = config.filter || {};

    this._gridElement.innerHTML = [
      `<section class="settings-card">${infoRow('Console', config.console_connected ? 'connected' : 'disconnected')}${infoRow('Heap', `${system.heap_bytes ?? 0} B`)}${infoRow('Uptime', `${system.uptime_ms ?? 0} ms`)}${infoRow('Wi-Fi client', system.wifi_client_connected ? 'connected' : 'none')}</section>`,
      `<section class="settings-card">${infoRow('Controller mode', controller.mode || 'unknown')}${infoRow('Adaptive trim', lqr.adaptive_trim_enabled ? 'enabled' : 'disabled')}${infoRow('Pitch LPF', `${asFixed(lqr.filters?.pitch_rate_lpf_hz, 2)} Hz`)}${infoRow('Cmd LPF', `${asFixed(lqr.filters?.cmd_lpf_hz, 2)} Hz`)}</section>`,
      `<section class="settings-card">${infoRow('Current filter', filter.current || 'unknown')}${infoRow('Supported params', (filter.supported || []).join(', ') || 'None')}${infoRow('Available filters', (filter.available || []).join(', ') || 'None')}</section>`,
      `<section class="settings-card">${infoRow('Driver', motors.driver || 'unknown')}${infoRow('Motors enabled', motors.enabled ? 'yes' : 'no')}${infoRow('Left inversion', motors.left?.inverted ? 'yes' : 'no')}${infoRow('Right inversion', motors.right?.inverted ? 'yes' : 'no')}</section>`,
      `<section class="settings-card">${infoRow('Enabled log channels', (logs.enabled_channels || []).join(', ') || 'None')}</section>`,
    ].join('');
  }
}

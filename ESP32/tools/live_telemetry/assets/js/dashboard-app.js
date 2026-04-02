import { DashboardApi } from './dashboard-api.js';
import { ChartManager } from './chart-manager.js';
import { MetricsPanel } from './metrics-panel.js';

export class DashboardApp {
  constructor(documentRef = document) {
    this._document = documentRef;
    this._api = new DashboardApi();
    this._chartManager = new ChartManager(
      this._document.getElementById('chartGrid'),
      this._document.getElementById('chartList'),
    );
    this._metricsPanel = new MetricsPanel(
      this._document.getElementById('status'),
      this._document.getElementById('metricGrid'),
    );
    this._windowLabelElement = this._document.getElementById('windowLabel');
    this._refreshSelectElement = this._document.getElementById('refreshSel');
    this._refreshHandle = null;
    this._refreshMs = 200;
  }

  start() {
    this._wireControls();
    this._chartManager.build();
    this._scheduleRefresh();
  }

  _wireControls() {
    this._document.getElementById('allBtn').addEventListener('click', () => {
      this._chartManager.showAll();
    });

    this._document.getElementById('noneBtn').addEventListener('click', () => {
      this._chartManager.hideAll();
    });

    this._refreshSelectElement.addEventListener('change', (event) => {
      this._refreshMs = Number(event.target.value);
      this._scheduleRefresh();
    });
  }

  _scheduleRefresh() {
    if (this._refreshHandle) {
      clearInterval(this._refreshHandle);
    }
    this._refreshHandle = setInterval(() => {
      void this._refresh();
    }, this._refreshMs);
    void this._refresh();
  }

  async _refresh() {
    try {
      const payload = await this._api.fetchState();
      this._windowLabelElement.textContent = payload.window_sec;
      this._metricsPanel.render(payload.latest, payload.status);
      this._chartManager.update(payload);
    } catch {
      this._metricsPanel.setDisconnected();
    }
  }
}

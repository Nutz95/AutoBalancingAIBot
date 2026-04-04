import { CapturePanel } from './capture-panel.js';
import { DashboardApi } from './dashboard-api.js';
import { ChartManager } from './chart-manager.js';
import { InfoPanel } from './info-panel.js';
import { LogsPanel } from './logs-panel.js';
import { MetricsPanel } from './metrics-panel.js';
import { SettingsPanel } from './settings-panel.js';
import { TabController } from './tab-controller.js';

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
    this._tabController = new TabController(this._document);
    this._settingsPanel = new SettingsPanel(this._document, this._api);
    this._logsPanel = new LogsPanel(this._document, this._api);
    this._infoPanel = new InfoPanel(this._document, this._api);
    this._capturePanel = new CapturePanel(this._document, this._api);
    this._windowLabelElement = this._document.getElementById('windowLabel');
    this._refreshSelectElement = this._document.getElementById('refreshSel');
    this._refreshHandle = null;
    this._refreshMs = 200;
    this._refreshInFlight = false;
  }

  start() {
    this._wireControls();
    this._tabController.start((tabName) => {
      this._handleTabChange(tabName);
    });
    this._settingsPanel.start();
    this._logsPanel.start();
    this._infoPanel.start();
    this._capturePanel.start();
    this._chartManager.build();
    this._tabController.activate('live');
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
    if (this._refreshInFlight) {
      return;
    }
    this._refreshInFlight = true;
    try {
      const payload = await this._api.fetchState();
      this._windowLabelElement.textContent = payload.window_sec;
      this._metricsPanel.render(payload.latest, payload.status);
      this._chartManager.update(payload);
    } catch {
      this._metricsPanel.setDisconnected();
    } finally {
      this._refreshInFlight = false;
    }
  }

  _handleTabChange(tabName) {
    if (tabName === 'settings') {
      void this._settingsPanel.load();
    }
    if (tabName === 'logs') {
      this._logsPanel.activate();
    } else {
      this._logsPanel.deactivate();
    }
    if (tabName === 'capture') {
      this._capturePanel.activate();
    } else {
      this._capturePanel.deactivate();
    }
    if (tabName === 'info') {
      void this._infoPanel.load();
    }
  }
}

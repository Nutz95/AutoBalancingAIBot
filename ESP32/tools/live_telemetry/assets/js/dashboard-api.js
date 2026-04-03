export class DashboardApi {
  async fetchState() {
    const response = await fetch('/api/state', { cache: 'no-store' });
    return response.json();
  }

  async fetchConfig(force = false) {
    const suffix = force ? '?force=1' : '';
    return this._getJson(`/api/config${suffix}`);
  }

  async refreshConfig() {
    const payload = await this._postJson('/api/config/refresh', {});
    return payload.config;
  }

  async fetchLogs(afterSequence = 0, limit = 200) {
    return this._getJson(`/api/logs?after=${afterSequence}&limit=${limit}`);
  }

  async updateControllerSettings(payload) {
    return this._postJson('/api/settings/controller', payload);
  }

  async updateFilterSettings(payload) {
    return this._postJson('/api/settings/filter', payload);
  }

  async setMotorsEnabled(enabled) {
    return this._postJson('/api/motors/enabled', { enabled });
  }

  async driveMotors(left, right) {
    return this._postJson('/api/motors/drive', { left, right });
  }

  async rebootSystem() {
    return this._postJson('/api/system/reboot', {});
  }

  async _getJson(url) {
    const response = await fetch(url, { cache: 'no-store' });
    const payload = await response.json();
    if (!response.ok || payload.ok === false) {
      throw new Error(payload.error || `Request failed with status ${response.status}`);
    }
    return payload;
  }

  async _postJson(url, payload) {
    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(payload),
    });
    const body = await response.json();
    if (!response.ok || body.ok === false) {
      throw new Error(body.error || `Request failed with status ${response.status}`);
    }
    return body;
  }
}

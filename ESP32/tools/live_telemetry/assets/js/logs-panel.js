export class LogsPanel {
  constructor(documentRef, api) {
    this._document = documentRef;
    this._api = api;
    this._terminalElement = this._document.getElementById('logsTerminal');
    this._statusElement = this._document.getElementById('logsStatus');
    this._refreshButton = this._document.getElementById('logsRefreshBtn');
    this._clearButton = this._document.getElementById('logsClearBtn');
    this._afterSequence = 0;
    this._pollHandle = null;
  }

  start() {
    this._refreshButton.addEventListener('click', () => {
      void this.refresh();
    });
    this._clearButton.addEventListener('click', () => {
      this._terminalElement.innerHTML = '';
    });
  }

  activate() {
    if (this._pollHandle) {
      return;
    }
    this._pollHandle = setInterval(() => {
      void this.refresh();
    }, 400);
    void this.refresh();
  }

  deactivate() {
    if (!this._pollHandle) {
      return;
    }
    clearInterval(this._pollHandle);
    this._pollHandle = null;
  }

  async refresh() {
    try {
      const payload = await this._api.fetchLogs(this._afterSequence);
      this._afterSequence = payload.latest_sequence || this._afterSequence;
      this._statusElement.textContent = payload.console_connected ? 'Console connected' : 'Console disconnected';
      for (const entry of payload.entries || []) {
        this._appendEntry(entry);
      }
      this._trimTerminal();
    } catch (error) {
      this._statusElement.textContent = error.message || 'Failed to fetch logs';
    }
  }

  _appendEntry(entry) {
    const lineElement = document.createElement('div');
    lineElement.className = `terminal-line level-${entry.level || 'robot'}`;
    lineElement.textContent = entry.message || '';
    this._terminalElement.appendChild(lineElement);
    this._terminalElement.scrollTop = this._terminalElement.scrollHeight;
  }

  _trimTerminal() {
    const maxLines = 500;
    while (this._terminalElement.childNodes.length > maxLines) {
      this._terminalElement.removeChild(this._terminalElement.firstChild);
    }
  }
}

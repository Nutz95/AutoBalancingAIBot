function setNumberInputValue(element, value) {
  if (!element) {
    return;
  }
  element.value = Number(value ?? 0).toFixed(6);
}

function setText(element, text) {
  if (element) {
    element.textContent = text;
  }
}

export class SettingsPanel {
  constructor(documentRef, api) {
    this._document = documentRef;
    this._api = api;
    this._statusElement = this._document.getElementById('settingsStatus');
    this._controllerForm = this._document.getElementById('controllerForm');
    this._filterForm = this._document.getElementById('filterForm');
    this._motorEnableButton = this._document.getElementById('motorEnableBtn');
    this._motorDisableButton = this._document.getElementById('motorDisableBtn');
    this._motorApplyButton = this._document.getElementById('motorDriveApplyBtn');
    this._motorStopButton = this._document.getElementById('motorDriveStopBtn');
    this._refreshButton = this._document.getElementById('settingsRefreshBtn');
    this._rebootButton = this._document.getElementById('rebootBtn');
    this._lastConfig = null;
  }

  start() {
    this._controllerForm.addEventListener('submit', (event) => {
      event.preventDefault();
      void this._saveControllerSettings();
    });

    this._filterForm.addEventListener('submit', (event) => {
      event.preventDefault();
      void this._saveFilterSettings();
    });

    this._motorEnableButton.addEventListener('click', () => {
      void this._setMotorsEnabled(true);
    });

    this._motorDisableButton.addEventListener('click', () => {
      void this._setMotorsEnabled(false);
    });

    this._motorApplyButton.addEventListener('click', () => {
      void this._applyMotorDrive();
    });

    this._motorStopButton.addEventListener('click', () => {
      void this._stopMotorDrive();
    });

    this._refreshButton.addEventListener('click', () => {
      void this.load(true);
    });

    this._rebootButton.addEventListener('click', () => {
      void this._rebootSystem();
    });
  }

  async load(forceRefresh = false) {
    this._setStatus('Refreshing settings…');
    try {
      const config = forceRefresh ? await this._api.refreshConfig() : await this._api.fetchConfig();
      this.applyConfig(config);
      this._setStatus('Settings synced');
    } catch (error) {
      this._setStatus(error.message || 'Failed to load settings', true);
    }
  }

  applyConfig(config) {
    this._lastConfig = config;
    const controller = config.controller || {};
    const pid = controller.pid || {};
    const lqr = controller.lqr || {};
    const lqrGains = lqr.gains || {};
    const lqrFilters = lqr.filters || {};
    const output = controller.output || {};
    const motorGains = output.motor_gains || {};
    const filter = config.filter || {};
    const motors = config.motors || {};

    const modeElement = this._document.getElementById('controllerMode');
    if (modeElement) {
      modeElement.value = controller.mode || 'LQR';
    }

    setNumberInputValue(this._document.getElementById('pidKp'), pid.kp);
    setNumberInputValue(this._document.getElementById('pidKi'), pid.ki);
    setNumberInputValue(this._document.getElementById('pidKd'), pid.kd);
    setNumberInputValue(this._document.getElementById('lqrKPitch'), lqrGains.k_pitch);
    setNumberInputValue(this._document.getElementById('lqrKGyro'), lqrGains.k_gyro);
    setNumberInputValue(this._document.getElementById('lqrKDist'), lqrGains.k_dist);
    setNumberInputValue(this._document.getElementById('lqrKSpeed'), lqrGains.k_speed);
    setNumberInputValue(this._document.getElementById('deadbandValue'), output.deadband);
    setNumberInputValue(this._document.getElementById('minCmdValue'), output.min_cmd);
    setNumberInputValue(this._document.getElementById('motorGainLeft'), motorGains.left);
    setNumberInputValue(this._document.getElementById('motorGainRight'), motorGains.right);
    setNumberInputValue(this._document.getElementById('pitchRateLpfHz'), lqrFilters.pitch_rate_lpf_hz);
    setNumberInputValue(this._document.getElementById('cmdLpfHz'), lqrFilters.cmd_lpf_hz);

    const adaptiveTrimElement = this._document.getElementById('adaptiveTrimEnabled');
    if (adaptiveTrimElement) {
      adaptiveTrimElement.checked = Boolean(lqr.adaptive_trim_enabled);
    }

    const filterSelectElement = this._document.getElementById('filterName');
    if (filterSelectElement) {
      filterSelectElement.innerHTML = (filter.available || []).map((name) => (
        `<option value="${name}">${name}</option>`
      )).join('');
      filterSelectElement.value = filter.current || '';
    }

    setText(this._document.getElementById('filterSupportedParams'), (filter.supported || []).join(', ') || 'None');
    setNumberInputValue(this._document.getElementById('filterAlpha'), filter.params?.ALPHA ?? 0);
    setNumberInputValue(this._document.getElementById('filterKacc'), filter.params?.KACC ?? 0);
    setNumberInputValue(this._document.getElementById('filterKbias'), filter.params?.KBIAS ?? 0);
    setNumberInputValue(this._document.getElementById('filterBeta'), filter.params?.BETA ?? 0);
    this._setFilterInputEnabled('filterAlpha', (filter.supported || []).includes('ALPHA'));
    this._setFilterInputEnabled('filterKacc', (filter.supported || []).includes('KACC'));
    this._setFilterInputEnabled('filterKbias', (filter.supported || []).includes('KBIAS'));
    this._setFilterInputEnabled('filterBeta', (filter.supported || []).includes('BETA'));

    setText(this._document.getElementById('motorsEnabledValue'), motors.enabled ? 'Enabled' : 'Disabled');
    setText(this._document.getElementById('motorDriverValue'), motors.driver || 'unknown');
    setText(this._document.getElementById('motorLeftMeta'), `ID ${motors.left?.id ?? '—'} · inverted: ${Boolean(motors.left?.inverted) ? 'yes' : 'no'}`);
    setText(this._document.getElementById('motorRightMeta'), `ID ${motors.right?.id ?? '—'} · inverted: ${Boolean(motors.right?.inverted) ? 'yes' : 'no'}`);
    setText(this._document.getElementById('motorLastCommands'), `Last commands L/R: ${Number(motors.left?.last_command ?? 0).toFixed(3)} / ${Number(motors.right?.last_command ?? 0).toFixed(3)}`);
  }

  async _saveControllerSettings() {
    this._setStatus('Saving controller settings…');
    try {
      const response = await this._api.updateControllerSettings(this._collectControllerPayload());
      await this.load(true);
      this._setStatus(response.ok ? 'Controller settings saved' : 'Controller settings updated');
    } catch (error) {
      this._setStatus(error.message || 'Failed to save controller settings', true);
    }
  }

  async _saveFilterSettings() {
    this._setStatus('Saving filter settings…');
    try {
      await this._api.updateFilterSettings(this._collectFilterPayload());
      await this.load(true);
      this._setStatus('Filter settings saved');
    } catch (error) {
      this._setStatus(error.message || 'Failed to save filter settings', true);
    }
  }

  async _setMotorsEnabled(enabled) {
    this._setStatus(enabled ? 'Enabling motors…' : 'Disabling motors…');
    try {
      await this._api.setMotorsEnabled(enabled);
      await this.load(true);
      this._setStatus(enabled ? 'Motors enabled' : 'Motors disabled');
    } catch (error) {
      this._setStatus(error.message || 'Failed to update motor state', true);
    }
  }

  async _applyMotorDrive() {
    this._setStatus('Sending motor commands…');
    try {
      const leftValue = Number(this._document.getElementById('motorDriveLeft').value || 0);
      const rightValue = Number(this._document.getElementById('motorDriveRight').value || 0);
      await this._api.driveMotors(leftValue, rightValue);
      await this.load(true);
      this._setStatus('Motor commands sent');
    } catch (error) {
      this._setStatus(error.message || 'Failed to drive motors', true);
    }
  }

  async _stopMotorDrive() {
    this._document.getElementById('motorDriveLeft').value = '0';
    this._document.getElementById('motorDriveRight').value = '0';
    await this._applyMotorDrive();
  }

  async _rebootSystem() {
    this._setStatus('Reboot requested…');
    try {
      await this._api.rebootSystem();
      this._setStatus('Reboot command sent');
    } catch (error) {
      this._setStatus(error.message || 'Failed to reboot robot', true);
    }
  }

  _collectControllerPayload() {
    return {
      mode: this._document.getElementById('controllerMode').value,
      pid: {
        kp: Number(this._document.getElementById('pidKp').value || 0),
        ki: Number(this._document.getElementById('pidKi').value || 0),
        kd: Number(this._document.getElementById('pidKd').value || 0),
      },
      lqr: {
        gains: {
          k_pitch: Number(this._document.getElementById('lqrKPitch').value || 0),
          k_gyro: Number(this._document.getElementById('lqrKGyro').value || 0),
          k_dist: Number(this._document.getElementById('lqrKDist').value || 0),
          k_speed: Number(this._document.getElementById('lqrKSpeed').value || 0),
        },
        adaptive_trim_enabled: this._document.getElementById('adaptiveTrimEnabled').checked,
        filters: {
          pitch_rate_lpf_hz: Number(this._document.getElementById('pitchRateLpfHz').value || 0),
          cmd_lpf_hz: Number(this._document.getElementById('cmdLpfHz').value || 0),
        },
      },
      output: {
        deadband: Number(this._document.getElementById('deadbandValue').value || 0),
        min_cmd: Number(this._document.getElementById('minCmdValue').value || 0),
        motor_gains: {
          left: Number(this._document.getElementById('motorGainLeft').value || 0),
          right: Number(this._document.getElementById('motorGainRight').value || 0),
        },
      },
    };
  }

  _collectFilterPayload() {
    const supportedParams = new Set(this._lastConfig?.filter?.supported || []);
    const params = {};
    if (supportedParams.has('ALPHA')) {
      params.ALPHA = Number(this._document.getElementById('filterAlpha').value || 0);
    }
    if (supportedParams.has('KACC')) {
      params.KACC = Number(this._document.getElementById('filterKacc').value || 0);
    }
    if (supportedParams.has('KBIAS')) {
      params.KBIAS = Number(this._document.getElementById('filterKbias').value || 0);
    }
    if (supportedParams.has('BETA')) {
      params.BETA = Number(this._document.getElementById('filterBeta').value || 0);
    }

    return {
      current: this._document.getElementById('filterName').value,
      params,
    };
  }

  _setFilterInputEnabled(elementId, enabled) {
    const element = this._document.getElementById(elementId);
    if (element) {
      element.disabled = !enabled;
    }
  }

  _setStatus(message, isError = false) {
    this._statusElement.textContent = message;
    this._statusElement.classList.toggle('is-error', isError);
  }
}

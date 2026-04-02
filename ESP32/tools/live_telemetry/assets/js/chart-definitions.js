export const chartDefs = {
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
    ],
  },
  lqr: {
    title: 'LQR decomposition',
    axisHints: [{ text: 'Left axis: LQR terms', color: '#4ea1ff' }],
    scaleConfig: { y: { title: 'LQR terms', color: '#4ea1ff', pad: 0.14, shrink: 0.18 } },
    series: [
      { key: 'lqr_angle', label: 'Angle', color: '#4ea1ff' },
      { key: 'lqr_gyro', label: 'Gyro', color: '#f0b84b' },
      { key: 'lqr_dist', label: 'Distance', color: '#ff6b6b' },
      { key: 'lqr_speed', label: 'Speed', color: '#24c37b' },
    ],
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
    ],
  },
  cpu: {
    title: 'CPU load',
    axisHints: [{ text: 'Left axis: CPU load %', color: '#4ea1ff' }],
    scaleConfig: { y: { title: 'CPU %', color: '#4ea1ff', min: 0, max: 100 } },
    series: [
      { key: 'cpu0_pct', label: 'Core0 %', color: '#4ea1ff' },
      { key: 'cpu1_pct', label: 'Core1 %', color: '#ff6b6b' },
    ],
  },
  rpm: {
    title: 'Motor RPM (RS485)',
    axisHints: [{ text: 'Left axis: motor RPM', color: '#4ea1ff' }],
    scaleConfig: { y: { title: 'RPM', color: '#4ea1ff', pad: 0.10, shrink: 0.14 } },
    series: [
      { key: 'motor_rpm_l', label: 'Left RPM', color: '#4ea1ff' },
      { key: 'motor_rpm_r', label: 'Right RPM', color: '#ff6b6b' },
    ],
  },
  step: {
    title: 'Applied STEP frequency',
    axisHints: [{ text: 'Left axis: applied STEP Hz', color: '#4ea1ff' }],
    scaleConfig: { y: { title: 'STEP Hz', color: '#4ea1ff', pad: 0.10, shrink: 0.14 } },
    series: [
      { key: 'step_hz_l', label: 'Left STEP Hz', color: '#4ea1ff' },
      { key: 'step_hz_r', label: 'Right STEP Hz', color: '#ff6b6b' },
    ],
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
    ],
  },
  imu: {
    title: 'Raw IMU axes',
    axisHints: [{ text: 'Left axis: raw IMU', color: '#4ea1ff' }],
    scaleConfig: { y: { title: 'IMU raw', color: '#4ea1ff', pad: 0.14, shrink: 0.16 } },
    series: [
      { key: 'gx', label: 'Gyro X', color: '#4ea1ff' },
      { key: 'gy', label: 'Gyro Y', color: '#ff8f5a' },
      { key: 'gz', label: 'Gyro Z', color: '#9d7dff' },
      { key: 'ax', label: 'Accel X', color: '#24c37b' },
      { key: 'ay', label: 'Accel Y', color: '#f0b84b' },
      { key: 'az', label: 'Accel Z', color: '#dbe7ff' },
    ],
  },
  mixing: {
    title: 'Command mixing / saturation',
    axisHints: [{ text: 'Left axis: commands / wheel mix', color: '#dbe7ff' }],
    scaleConfig: { y: { title: 'Mix / command', color: '#dbe7ff', pad: 0.14, shrink: 0.16 } },
    series: [
      { key: 'cmd_raw', label: 'cmd raw', color: '#dbe7ff' },
      { key: 'cmd_final', label: 'cmd final', color: '#24c37b' },
      { key: 'left_preclip', label: 'left preclip', color: '#4ea1ff' },
      { key: 'right_preclip', label: 'right preclip', color: '#ff6b6b' },
      { key: 'left_postclip', label: 'left postclip', color: '#4ea1ff', dashed: true },
      { key: 'right_postclip', label: 'right postclip', color: '#ff6b6b', dashed: true },
    ],
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
    ],
  },
  fft: {
    title: 'Live FFT / oscillations',
    source: 'fft',
    axisHints: [
      { text: 'X axis: frequency (Hz)', color: '#4ea1ff' },
      { text: 'Y axis: relative magnitude', color: '#dbe7ff' },
    ],
    scaleConfig: { y: { title: 'Magnitude', color: '#dbe7ff', pad: 0.18, shrink: 0.18, quantileMin: 0.0, quantileMax: 1.0 } },
    series: [
      { key: 'pitch_mag', label: 'Pitch FFT', color: '#4ea1ff' },
      { key: 'cmd_mag', label: 'Command FFT', color: '#24c37b' },
      { key: 'rpm_mag', label: 'RPM FFT', color: '#ff8f5a' },
    ],
  },
};

export const defaultCharts = Object.keys(chartDefs);

# Tuning Telemetry (TUNING) — Quick Start

This short README explains how to use the `TUNING` CSV telemetry mode implemented in firmware.

Purpose
- Provide a clean, machine-readable CSV stream with fused IMU outputs (pitch + pitch rate) and motor commands.

How to enable
- Open a serial console to the board (native USB) at `921600` baud.
- Send the command: `TUNING START` to begin CSV emission. The firmware will print a CSV header first.
- To stop: `TUNING STOP`.

CSV format
- Header (example):
  - `timestamp_ms,pitch_deg,pitch_rad,pitch_rate_deg,pitch_rate_rad,ax,ay,az,gx,gy,gz,temp_C,left_cmd,right_cmd`
- Rows: one CSV row per IMU cycle (sample rate derived from BMI088 sampling).


Startup warmup and safety sequence
----------------------------------
When the board boots, the IMU fusion (Madgwick) module performs a warmup phase:
- The system collects a configurable number of IMU samples (default: ~200, ≈1s) to estimate initial attitude and gyro bias.
- During this phase, the status LED is set to **red**. The balancer cannot be started until warmup completes.
- Once warmup is done, the LED turns **green** and the system is ready for balancing (`fusion.isReady()` returns true).
- The balancer startup (`BALANCE START`) is gated: motors will not be enabled until fusion is ready. A CLI override is available for advanced users.

This ensures safe startup and prevents the motors from activating with uninitialized or biased attitude estimates.

Recommended bench procedure
1. Ensure the robot is securely mounted on a stable fixture where it cannot fall or cause damage.
2. Power the system and confirm BMI088 shows reasonable gravity on one axis.
3. Wait for the status LED to turn green (warmup complete) before issuing any balancing or tuning commands.
4. Open serial monitor: `pio device monitor --baud 921600` (or use your terminal/serial app).
5. Send `TUNING START` and capture the output to a file (or copy/paste). Example using `pio device monitor` piping is platform-specific — use your terminal's copy or a serial logger.
6. Analyze the `pitch_deg` column: compute mean and stddev. Expected: stddev < 0.5° when stable and mean within ±2°.

Beta tuning guidance
- The `beta` parameter controls Madgwick responsiveness. Suggested starting range: `0.08`–`0.12`.
- If the estimate is noisy, decrease `beta` slightly; if it lags real motion, increase it.

Post-processing hints
- Use a simple Python snippet to parse CSV and compute statistics (mean/stddev) on `pitch_deg`.

Safety
- Always have a power cutoff and avoid enabling motors when testing stationary IMU fusion.

Files
- `openspec/changes/add-imufusion-madgwick/bench_report.md` — optional location to store bench results.

Artifacts & archive
- After running captures and analysis, archive your CSV and PNGs under the openspec change for traceability. Example destination used in this repo:
  - `openspec/changes/add-imufusion-madgwick/artifacts/plots/`
  - A manifest of archived files is available at `openspec/changes/add-imufusion-madgwick/artifacts/manifest.txt`

Note: the recent 10k-sample capture used for benching produced a `TUNING SUMMARY` and plots archived under the paths above; see `openspec/changes/add-imufusion-madgwick/artifacts/summary.txt` for the numeric summary.

TUNING SUMMARY (firmware)
- Example firmware output (appears after `TUNING STOP` or when auto-capture completes):
  - `TUNING SUMMARY: samples=2000, pitch_mean=0.185635, pitch_std=0.085277, pr_mean=0.025836, pr_std=0.136723`
  - `Suggested beta ~ 0.084`

What the fields mean
- `samples`: number of samples collected.
- `pitch_mean`: mean pitch over the capture (degrees when CSV column is `pitch_deg`).
- `pitch_std`: standard deviation of pitch (degrees) — an indicator of angular noise/drift.
- `pr_mean`: mean pitch-rate (degrees/sec when CSV column is `pitch_rate_deg`).
- `pr_std`: standard deviation of pitch-rate (degrees/sec) — indicates angular velocity noise.
- `Suggested beta`: heuristic suggestion from firmware to seed Madgwick's `beta` gain; treat it as a starting point, not a final tuned value.

Quick interpretation (example values above)
- `pitch_mean ~= 0.19°` — very close to zero; good for a stationary bench test.
- `pitch_std ~= 0.085°` — small angular noise; indicates a stable IMU reading on the bench.
- `pr_std ~= 0.137°/s` — modest angular-rate noise.
- `Suggested beta ~ 0.084` — consistent with the low-noise measurements; reasonable starting point. If your system feels sluggish during motion, slightly increase `beta`. If it jitters/noisy, decrease `beta`.

Recommended tuning workflow
1. Run a capture using the host helper so the serial stream is saved to a file. Preferred helper (from repo):
   - `ESP32/tools/capture_tuning.ps1` (PowerShell wrapper) — it will prefer the Python helper if available and defaults to `TUNING START 2000` when run without arguments.
   - Or run: `python ESP32/tools/capture_tuning.py -p COMX -b 921600 -c "TUNING START 2000"` to capture to a timestamped file.
2. Examine the `TUNING SUMMARY` printed by firmware at the end. Use the `pitch_std` and `pr_std` to assess noise levels and to choose a `beta` seed.
3. Try the suggested beta, then test dynamic motion: small nudges and observe latency vs noise. Adjust in small steps (±10–30%).

Notes and assumptions
- The example assumes the CSV columns indicated earlier (`pitch_deg`, `pitch_rate_deg`). If you are using radians columns, convert appropriately.
- The firmware `Suggested beta` is a heuristic. Final tuning should be done with real-world motion tests (balancing, step responses, disturbance rejection).

FRANÇAIS — Mode d'emploi et interprétation rapide
- Le résumé `TUNING SUMMARY` donne les moyennes et écarts-types calculés pendant la capture (Welford en une passe).
- Dans l'exemple ci‑dessus, la moyenne d'assiette (`pitch_mean`) est proche de 0° et l'écart-type (`pitch_std`) est bas — bon signe pour un test statique sur banc.
- `pr_std` (écart‑type de la vitesse angulaire) indique le bruit en vitesse; une valeur faible signifie que l'estimation du taux est propre.
- La `Suggested beta` fournie par le firmware est un point de départ. Testez-la, puis augmentez si l'estimation traîne derrière les mouvements réels, ou diminuez si elle devient trop bruitée.

Souhaitez-vous que j'ajoute un petit script Python d'analyse (moyenne, std, PSD rapide) qui prend le fichier de capture et propose un balayage de `beta` pour comparaison ?

Auto-persistence du biais gyro (nouveau)
- Le firmware collecte maintenant un biais gyro initial pendant la phase de warm-up, mais NE l'écrit PAS immédiatement dans la NVS.
- Au lieu de cela il retient un candidat de biais et attend une courte période (par défaut 5 s) de stationnarité avant d'écrire dans l'espace `abbot` (`gbx,gby,gbz`).
- Ce comportement évite de stocker un biais mesuré pendant que vous manipulez la platine immédiatement après le démarrage.

Balancer / PID — prochaines étapes pour activer les moteurs
- Le projet contient maintenant un squelette de contrôleur PID séparé dans `ESP32/src/balancer_controller.h/.cpp`.
- Étapes recommandées pour activer les moteurs en toute sécurité :
  - Vérifier le câblage moteur / servo et la configuration dans `ESP32/config/motor_config.h` (IDs, pins, inversion).
  - Tester la commande manuelle des moteurs (via `MOTOR ENABLE` puis `MOTOR SET LEFT 0.2` et `MOTOR SET RIGHT 0.2`) pour vérifier la réponse et la direction.
  - Intégrer le contrôleur PID : implémenter une routine qui lit `abbot::getFusedPitch()` et `abbot::getFusedPitchRate()` puis calcule une commande à l'aide de `abbot::balancer::PIDController::update(error, error_dot, dt)`.
  - Ne pas activer les moteurs automatiquement au démarrage. Utiliser d'abord la commande série `MOTOR ENABLE` depuis un terminal pour armer les moteurs.

Architecture suggérée pour le PID
- Séparer les responsabilités :
  - `balancer_controller` : calcul du PID, anti-windup, limites.
  - `SystemTasks` : lecture IMU, fusion Madgwick, appel au balancer pour obtenir la consigne moteur.
  - `motor_driver` : API d'abstraction matérielle (déjà existante).

Consignes de sécurité avant essai dynamique
- Montez le robot sur un support qui empêche toute chute.
- Préparez un interrupteur d'alimentation ou un bouton d'arrêt rapide.
- Commencez avec gains PID faibles (Kp petit, Ki=0, Kd petit) et validez la réponse en poussant légèrement.

Souhaitez-vous que j'intègre un exemple minimal d'appel au PID dans `SystemTasks` (désactivé par défaut par un `#define`), ou que je crée un petit guide pas-à-pas pour les tests moteurs ?

Motor acceleration (runtime) control
 - The firmware now exposes a runtime command to adjust servo acceleration without rebuilding: `MOTOR ACC <LEFT|RIGHT|ID> <value>`
 - Example: `MOTOR ACC LEFT 0` (sets acceleration register to 0 — fastest changes, higher current spikes)
 - Example: `MOTOR ACC LEFT 200` (sets a smoothing acceleration value; this repository's default is `200`)
 - The compile-time defaults are configured in `ESP32/config/motor_config.h`:
   - `VELOCITY_MAX_SPEED` (default `7000`) — raw servo units used to map normalized velocity commands
   - `MOTOR_SERVO_DEFAULT_ACC` (default `200`) — default acceleration register written on enable
 - Use the runtime `MOTOR ACC` command during bench testing to quickly switch between aggressive and smooth acceleration settings. Monitor servo current and temperature when trying aggressive values (e.g. `0`) — higher currents and heating are expected under load.

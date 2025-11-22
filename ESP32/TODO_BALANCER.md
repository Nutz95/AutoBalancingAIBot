<!-- TODO list for building the self-balancing robot -->
# TODO: Robot Auto-équilibrage (Balancer)

Ce fichier liste les tâches pratiques, leur ordre recommandé, les critères d'acceptation et les fichiers concernés.

## Contexte

- Moteurs: STS3215HS (servos compatibles SCServo workload)
- Librairie prévue: `workloads/SCServo` (déjà déclarée dans `waveshare_resources/.../platformio.ini`)
- Capteurs: BMI088 (accel + gyro) via SPI (drivers et diagnostics déjà intégrés)

---

## Priorité et ordre recommandé

1. Collecte: lectures statiques (orientation)
2. Calibration IMU (gyro bias, accel offsets)
3. Remap axes (configuration)
4. Filtre de fusion (estimateur d'angle)
5. API estimateur d'état (pitch, pitch_rate)
6. Intégration moteurs: STS3215HS via `workloads/SCServo`
7. Contrôleur d'équilibrage (inner PID)
8. Encodeurs + boucle extérieure (position/vitesse)
9. Sécurité & watchdog
10. Logging / télémetrie pour tuning
11. Tests banc et tuning progressif
12. Documentation finale & CI

---

## Checklist détaillée (tâches, emplacement, critères d'acceptation)

- **Collecte: orientation statique** (dossier: `ESP32/test/orientation_logs/`)
  - Action: placer le robot dans 6 orientations (plat / tête en haut / tête en bas / gauche / droite / avant / arrière) et capturer ~5 lignes Serial par position.
  - But: déterminer mapping axes et signes (quel axe correspond au pitch et son signe).
  - Critère: présence de logs pour chaque orientation et confirmation de l'axe de pitch.

- **Calibration IMU (startup)**
  - Fichier: `ESP32/src/imu_calibration.cpp` (nouveau)
  - Action: calculer bias gyro (N=500..2000 échantillons immobiles) et offsets accel; stocker en flash/EEPROM.
  - Critère: dérive gyro à repos < quelques deg/s; accel statique ≈ ±9.8 m/s² après offset.

- **Axis remap layer**
  - Fichier: `ESP32/include/BMI088Config.h` (ajout de flags `remap_x`, `remap_y`, `remap_z`, `swap_xy`, etc.)
  - Action: appliquer remap dans `ESP32/src/BMI088Driver.cpp` après lecture brut.
  - Critère: tests statiques montrent pitch correct après remap.

- **Filtre de fusion (imu_fusion)**
  - Fichiers: `ESP32/src/imu_fusion.cpp`, `ESP32/include/imu_fusion.h`
  - Option simple: filtre complémentaire (alpha ≈ 0.98) ; option avancée: Madgwick/Mahony.
  - Fréquence: viser 200Hz (ou 500Hz si CPU le permet).
  - Critère: angle stable à repos (< ±2° d'erreur) et réponse cohérente au mouvement.

- **API estimateur d'état**
  - Fournir `getPitch()` et `getPitchRate()` (et timestamp) pour le contrôleur.

- **Intégration moteurs: STS3215HS + SCServo**
  - Fichiers: `ESP32/src/motor_driver.cpp`, `ESP32/include/motor_driver.h`
  - Utiliser `workloads/SCServo` pour: commander les servos, lire encodeurs intégrés, gestion des modes.
  - Fonctions requises: `enableMotors()`, `disableMotors()`, `setMotorCommand(id, pwm)`, `readEncoder(id)`.
  - Sécurité: limiter PWM max par configuration (`max_pwm_percent`) et ramping.
  - Critère: moteurs répondent aux commandes en banc (sans roues) et encodeurs retournent counts plausibles.

- **Contrôleur d'équilibrage (inner loop)**
  - Fichier: `ESP32/src/balance_controller.cpp` + header
  - Algorithme: PID (anti-windup) → consigne angle → commande moteurs (PWM)
  - Fréquence: 100–200Hz (200Hz recommandé)
  - Critère: sur banc de test, petites perturbations corrigées et stabilité initiale atteinte.

- **Encodeurs + boucle extérieure (position/vitesse)**
  - Utiliser retours encoders SCServo pour estimer vitesse de roue et position.
  - Implémenter boucle extérieure pour consigne de déplacement (PID ou PI).

- **Sécurité & watchdog**
  - Ajouter: bouton d'arrêt hardware, angle cutoff (ex: 30°), watchdog logiciel 500ms.
  - Critère: dépassement angle → coupure immédiate moteurs.

- **Logging & tuning harness**
  - Mode `TUNING`: envoyer CSV via Serial (921600) contenant timestamp, pitch, pitch_rate, motor_cmd, encoder
  - Stocker logs dans `ESP32/test/tuning_logs/` si SD disponible.

- **Bench testing & tuning**
  - Procédure: tests sur banc → supports → essais réels; itérer gains et filtrages.

- **Documentation & CI**
  - Mettre à jour `openspec/project.md`, ajouter `ESP32/README_TUNING.md` et instructions de test.

---

## Commandes utiles (PowerShell, depuis `ESP32`)

```powershell
# Build & upload
pio run --target clean; pio run --target upload

# Moniteur série
pio device monitor --baud 921600
```

---

## Notes spécifiques: STS3215HS + SCServo

- STS3215HS est un servo avec feedback/encodeur gérable via la librairie SCServo.
- `workloads/SCServo` doit être présent dans `platformio.ini` (déjà référencé dans `waveshare_resources/.../platformio.ini`).
- Assurer que l'ID/addr des servos et la vitesse de communication (baud) sont configurés dans `motor_driver` au démarrage.

---

Si tu veux, je peux commencer par:

A) ajouter le squelette `motor_driver` qui initialise `SCServo` et lit les encodeurs (recommandé)

B) implémenter la couche `imu_fusion` (complémentaire)

C) ajouter la routine de calibration IMU et l'UI Serial pour lancer la calibration

Indique A, B ou C et je commence l'implémentation correspondante.

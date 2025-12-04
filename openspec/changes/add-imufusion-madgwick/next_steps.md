# Next Steps — IMU Fusion (Madgwick) & TUNING captures

Contexte
- Capture `tuning_capture_10000` (stationary, robot vertical on support) a été réalisée et l'analyse est archivée dans `artifacts/summary.txt`.

Recommandations immédiates
1. Archiver les PNGs générés par `analyze_tuning_capture.py` dans `openspec/changes/add-imufusion-madgwick/artifacts/plots/` pour traçabilité.
2. Réaliser une capture en mode BALANCER actif (si sûr) pour collecter les réponses en boucle fermée (commande moteur vs angle). Exemple de commande à envoyer : `TUNING START 10000` puis `BALANCE START` après warmup/header.
3. Tester une valeur de Madgwick beta plus petite (0.08–0.12) et refaire une capture stationnaire pour mesurer l'impact sur `pitch_std`.
4. Préparer une série d'expériences pour PID (Direct Velocity Mode) :
   - Fixer beta recommandé (ex. 0.10).
   - Captures: (a) stationnaire 10k ; (b) balancer actif 10k ; (c) step tests sur consigne de vitesse.
   - Générer plots et RMSE entre accel-derived angle et fused angle.
5. Documenter les résultats et ouvrir une PR `openspec/changes/add-imufusion-madgwick` contenant :
   - `bench_report.md` (mis à jour),
   - `artifacts/summary.txt`,
   - `artifacts/plots/*.png` (si acceptés),
   - `next_steps.md` (ce fichier).

Sécurité & contenu
- Les captures forment la preuve expérimentale; ne pas engager le balancer avec personnes près si les commandes moteurs sont actives.
- Les captures brutes restent dans `ESP32/tools/tuning_capture_10000/` — on peut dupliquer les fichiers importants dans `openspec` pour archivage.

Demande pour vous
- Voulez-vous que j'ajoute les PNGs produits par `analyze_tuning_capture.py` au répertoire `artifacts/plots/` et que je commette tout cela ?
- Ou préférez-vous que je prépare une PR avec seulement `bench_report.md`, `artifacts/summary.txt` et `next_steps.md` pour revue ?

# AutoBalancingAIBot — ESP32 Starter

This folder contains a minimal PlatformIO starter project for the ESP32-S3 used by the AutoBalancingAIBot project.

Build

Open a terminal in this folder and run:

```powershell
pio run
```

Upload (after configuring `upload_port` in `platformio.ini` if needed):

```powershell
pio run --target upload
```

Serial monitor:

```powershell
pio device monitor --baud 115200
```

This project prints a heartbeat on serial every second.

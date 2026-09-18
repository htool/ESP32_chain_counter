# PlatformIO extra script: compile-time include of gitignored wifi_secrets.h
# at the project root (not under src/). Missing file → AP provisioning.

from pathlib import Path

Import("env")  # type: ignore  # PlatformIO

secrets = Path(env["PROJECT_DIR"]) / "wifi_secrets.h"
if secrets.is_file():
    env.Append(CCFLAGS=["-include", str(secrets)])
    print(f"Including Wi-Fi secrets from {secrets}")
else:
    print("No wifi_secrets.h; firmware will start a config AP")

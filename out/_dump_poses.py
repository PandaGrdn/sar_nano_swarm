#!/usr/bin/env python3
"""Dump one odom + IMU sample per cf. Filename avoids pkill patterns."""
import subprocess
import sys
import time

def echo(topic: str, timeout: float = 4.0) -> str:
    try:
        r = subprocess.run(
            ["timeout", str(int(timeout)), "ros2", "topic", "echo", "--once", topic],
            capture_output=True, text=True, timeout=timeout + 2,
        )
        return (r.stdout or "") + (r.stderr or "")
    except Exception as e:
        return f"ERR {e}"

for i in range(3):
    print(f"===== /cf_{i}/odom =====")
    print(echo(f"/cf_{i}/odom")[:900])
    print(f"===== /cf_{i}/imu (accel/gyro) =====")
    print(echo(f"/cf_{i}/imu")[:900])
    time.sleep(0.2)

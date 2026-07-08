#!/usr/bin/env bash
set -euo pipefail

TOPIC="${1:-/radar/points}"
TIMEOUT_SEC="${2:-15}"

echo "Waiting up to ${TIMEOUT_SEC}s for ${TOPIC} ..."
if timeout "${TIMEOUT_SEC}" ros2 topic echo "${TOPIC}" sensor_msgs/msg/PointCloud2 --once >/dev/null; then
  echo "OK: received one ${TOPIC} message"
  ros2 topic hz "${TOPIC}" --window 5 || true
  exit 0
fi

echo "ERROR: no ${TOPIC} messages received within ${TIMEOUT_SEC}s" >&2
exit 1

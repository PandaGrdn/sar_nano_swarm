#!/usr/bin/env bash
# Build radarays_gz2 into repo install/ so phase0 loads Doppler fields.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
# shellcheck disable=SC1091
source setup_env.sh
BUILD="$ROOT/perception/radarays_gz2/build"
PREFIX="$ROOT/install/radarays_gz2"
mkdir -p "$BUILD"
cmake -S "$ROOT/perception/radarays_gz2" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build "$BUILD" -j"$(nproc)"
cmake --install "$BUILD"
echo "[build] $PREFIX/lib/libradar_sensor_system.so"
ls -l "$PREFIX/lib/libradar_sensor_system.so"
# Keep crazyflie_ws copy in sync if that tree exists (old RADAR_PLUGIN_DIR).
_CF="$HOME/crazyflie_ws/install/radarays_gz2/lib"
if [[ -d "$_CF" ]]; then
  cp -f "$PREFIX/lib/libradar_sensor_system.so" "$_CF/"
  echo "[build] also copied to $_CF"
fi

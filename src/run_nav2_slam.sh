#!/usr/bin/env bash
set -euo pipefail

# Convenience wrapper so you can run Nav2+SLAM from the `src/` folder.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT_DIR/scripts/run_nav2_slam.sh"


#!/usr/bin/env bash
set -euo pipefail
tracker_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$tracker_dir/software/led_gui.py" "$@"

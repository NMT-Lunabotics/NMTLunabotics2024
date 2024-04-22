#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

python3 -m cantools monitor -c can0 main_bus.kcd

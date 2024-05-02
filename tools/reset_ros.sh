#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# This assumes our current robot deployment: `goliath` is installed
# into $HOME, as is `ros4nix`.

cd ~/ros4nix
git pull || true                # allow failure in case of no internet
cp ~/goliath/*.nix .
./ros4nix switch configuration.nix

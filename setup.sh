#!/bin/sh
# Creates the virtual environment in .venv and installs the dependencies into it.
set -e
cd "$(dirname "$0")"
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt
echo "Done. Run ./wpi2mcap --help"

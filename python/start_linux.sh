#!/usr/bin/env bash
# Run from VS Code terminal or Linux shell; main.py provisions its own runtime.
set -e
cd "$(dirname "$0")"
exec python3 main.py "$@"

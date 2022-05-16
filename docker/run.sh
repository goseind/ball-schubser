#!/bin/bash
set -e

echo "starting main.py and watching for file changes ..."

watchmedo auto-restart --patterns="*.py" --recursive python3 /app/main.py

#!/bin/bash

# Usage: `./scripts/clean.sh`

rm -rf build install log

if [ $? -eq 0 ]; then
    echo "Clean complete. To rebuild the workspace, run: ./scripts/build.sh"
else
    echo "Clean failed."
    exit 1
fi
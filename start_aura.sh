#!/bin/bash
echo "Starting AURA System..."
# The --build flag is the critical fix.
# It forces docker compose to rebuild the image if the Dockerfile has changed.
docker compose up -d --build
echo "AURA is running in the background."
#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

cd /app

if [ "${COLCON_BUILD_ON_START:-0}" = "1" ] && [ -f package.xml ]; then
  colcon build --symlink-install
fi

if [ -f /app/install/setup.bash ]; then
  source /app/install/setup.bash
fi

exec "$@"

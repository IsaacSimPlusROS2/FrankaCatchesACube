#!/usr/bin/env bash
set -euo pipefail

ISAACSIM=/home/mryan2005/isaac-sim-standalone-5.1.0-linux-x86_64
WORLD=/home/mryan2005/frankaSpace/world.py

# 开一个最小环境，避免 conda/ROS 的路径渗入
env -i \
  HOME="$HOME" \
  PATH="/usr/bin:/bin:/usr/sbin:/sbin" \
  ROS_DOMAIN_ID="0" \
  "$ISAACSIM/python.sh" "$WORLD"
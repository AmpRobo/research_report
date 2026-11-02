#!/usr/bin/env bash
set -euo pipefail

if ! pkexec /usr/bin/docker exec ubuntu2204 bash -lc \
  'pids=$(pgrep -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch malecns_robot demo.launch.py" || true); if [ -n "$pids" ]; then kill -INT $pids; fi'; then
  zenity --error --title='关闭仿真失败' --text='请确认 Docker 已启动、容器存在，并完成管理员授权。' --width=420
  exit 1
fi
zenity --info --title='MaleCNS 机器人仿真' --text='已请求关闭 Gazebo、RViz 和机器人控制节点。' --width=420

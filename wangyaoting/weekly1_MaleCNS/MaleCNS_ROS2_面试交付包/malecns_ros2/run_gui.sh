#!/usr/bin/env bash
set -euo pipefail
sudo systemctl start docker
sudo docker start ubuntu2204 >/dev/null 2>&1 || true
xhost +si:localuser:root >/dev/null
sudo docker exec -it -e DISPLAY="${DISPLAY:-:0}" -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 ubuntu2204 \
  bash -lc 'source /opt/ros/humble/setup.bash; source /root/malecns_ws/install/setup.bash; ros2 launch malecns_robot demo.launch.py gui:=true rviz:=true'

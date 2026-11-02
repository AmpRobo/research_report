#!/usr/bin/env bash
set -euo pipefail

dialog() {
  zenity --info --title='MaleCNS 机器人仿真' --text="$1" --width=420
}
fail() {
  zenity --error --title='仿真启动失败' --text="$1" --width=450
  exit 1
}

[[ -n "${DISPLAY:-}" ]] || fail '请先登录这台电脑的图形桌面，再双击启动图标。'
xhost +si:localuser:root >/dev/null 2>&1 || fail '无法连接图形桌面，请检查 DISPLAY 和 X11 会话。'

pkexec /usr/bin/systemctl start docker || fail 'Docker 启动失败。请确认管理员授权窗口中的操作。'
pkexec /usr/bin/docker start ubuntu2204 >/dev/null || fail '找不到或无法启动 ubuntu2204 容器。'

if pkexec /usr/bin/docker exec ubuntu2204 bash -lc \
  'pgrep -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch malecns_robot demo.launch.py" >/dev/null'; then
  dialog '仿真已经在运行。请切换到 Gazebo 和 RViz 窗口查看。'
  exit 0
fi

pkexec /usr/bin/docker exec -d \
  -e "DISPLAY=${DISPLAY}" \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e QT_X11_NO_MITSHM=1 \
  -e OPENBLAS_NUM_THREADS=1 \
  -e GAZEBO_MODEL_DATABASE_URI= \
  ubuntu2204 bash -lc \
  'source /opt/ros/humble/setup.bash && source /root/malecns_ws/install/setup.bash && exec ros2 launch malecns_robot demo.launch.py gui:=true rviz:=true log_path:=/tmp/malecns_click_run.csv > /tmp/malecns_click_run.log 2>&1' \
  || fail 'ROS2 启动失败。请查看容器内 /tmp/malecns_click_run.log。'

dialog '已开始启动。Gazebo 和 RViz 窗口通常会在十几秒内出现；机器人会自动朝默认目标前进。'

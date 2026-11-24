#!/usr/bin/env bash
# 用 Windows 上的仿真视频流启动 ROS，地址自动解析，不用手填 IP。
set -eo pipefail
cd "$(dirname "$0")/../.."
if [[ ${1:-} == --start || ${1:-} == --reset ]]; then
  export ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1
  [[ $1 == --start ]] && action=start-task || action=reset-task
  exec bash tools/preview.sh "$action"
fi
host=$(ip route show default | awk '{print $3}')
if [[ -z $host ]]; then
  echo "[run_sim] 找不到 Windows 主机地址，确认 WSL 网络正常" >&2
  exit 1
fi
if [[ ${1:-} == --desktop ]]; then
  team=${2:-red}
  [[ $team == red || $team == blue ]] || exit 2
  model="$PWD/assets/models/train5.onnx"
  if [[ ! -r $model || ! -r install/local_setup.bash ]]; then
    echo '[run_sim] 需要 train5.onnx 和编译好的 ROS 工作区' >&2
    exit 1
  fi
  export ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1
  export PYTHONUNBUFFERED=1
  export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/tools/preview_fastdds.xml"
  source /opt/ros/humble/setup.bash
  source install/local_setup.bash
  # 沿用原电脑仿真的追踪增益，打散仍使用单独的速度映射。
  coproc STM32 {
    exec python3 tools/stm32_sim.py --device-only --ros-clock --sim-host "$host" \
      --sim-port "${SIM_CONTROL_PORT:-8091}" --speed-gain .0004 --turn-gain .0006 \
      --speed-limit .12 --turn-limit .8
  }
  serial_pid=$STM32_PID
  launch_pid=
  trap 'if [[ -n $launch_pid ]]; then kill -INT -- "-$launch_pid" 2>/dev/null || true; wait "$launch_pid" || true; fi; kill "$serial_pid" 2>/dev/null || true' EXIT
  read -r -t 5 uart <&"${STM32[0]}"
  [[ $uart == /dev/pts/* ]] || exit 1
  setsid bash tools/run.sh source:=http camera_url:="http://$host:${SIM_PORT:-8090}/video" \
    "model:=$model" "uart:=$uart" "team:=$team" port:=18082 preview_width:=1280 preview_height:=720 \
    safety_detector:=true vision_timeout_s:=1.2 sim_distribute:=true &
  launch_pid=$!
  # Windows 客户端通过关闭标准输入请求停止，ROS 和伪串口跟着退出。
  (IFS= read -r _; kill -INT -- "-$launch_pid" 2>/dev/null || true) <&0 &
  wait "$launch_pid"
  exit
fi
exec bash tools/run.sh source:=http camera_url:="http://$host:${SIM_PORT:-8090}/video" "$@"

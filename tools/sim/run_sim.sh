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
  model="$PWD/assets/models/claw_20261125.onnx"
  [[ -r $model ]] || model="$PWD/assets/models/train5.onnx"
  if [[ ! -r $model || ! -r install/local_setup.bash ]]; then
    echo '[run_sim] 需要 ONNX 模型和编译好的 ROS 工作区' >&2
    exit 1
  fi
  export ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1
  export PYTHONUNBUFFERED=1
  export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/tools/preview_fastdds.xml"
  source /opt/ros/humble/setup.bash
  source install/local_setup.bash
  # 追踪用较快的速度映射，转向超过死区后给够起转速度。打散仍用独立映射。
  coproc STM32 {
    exec python3 tools/stm32_sim.py --device-only --ros-clock --sim-host "$host" \
      --sim-port "${SIM_CONTROL_PORT:-8091}" --speed-gain .00075 --turn-gain .0028 \
      --speed-limit .12 --turn-limit .8 --min-turn .22 --angle-turn-limit .9
  }
  serial_pid=$STM32_PID
  launch_pid=
  trap 'if [[ -n $launch_pid ]]; then kill -INT -- "-$launch_pid" 2>/dev/null || true; wait "$launch_pid" || true; fi; kill "$serial_pid" 2>/dev/null || true' EXIT
  read -r -t 30 uart <&"${STM32[0]}"
  [[ $uart == /dev/pts/* ]] || exit 1
  setsid bash tools/run.sh source:=http camera_url:="http://$host:${SIM_PORT:-8090}/video" \
    "model:=$model" "uart:=$uart" "team:=$team" port:=18082 preview_width:=640 preview_height:=480 \
    safety_detector:=true vision_timeout_s:=1.8 sim_distribute:=true &
  launch_pid=$!
  # Windows 客户端通过关闭标准输入请求停止，ROS 和伪串口跟着退出。
  (IFS= read -r _; kill -INT -- "-$launch_pid" 2>/dev/null || true) <&0 &
  wait "$launch_pid"
  exit
fi
exec bash tools/run.sh source:=http camera_url:="http://$host:${SIM_PORT:-8090}/video" "$@"

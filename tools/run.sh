#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/local_setup.bash
if [[ -x tools/.venv/bin/python3 ]]; then
  export VIRTUAL_ENV="$PWD/tools/.venv"
  export PATH="$VIRTUAL_ENV/bin:$PATH"
fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/tools/logs/ros}"
sim_pid=
serial_pid=
cleanup() {
  for pid in "$serial_pid" "$sim_pid"; do
    if [[ -n $pid ]]; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}

start_stm32_sim() {
  coproc STM32_SIM { exec python3 tools/stm32_sim.py --device-only; }
  sim_pid=$STM32_SIM_PID
  trap cleanup EXIT
  if ! read -r -t 5 uart <&"${STM32_SIM[0]}"; then
    echo 'STM32 模拟器未提供串口设备' >&2
    exit 1
  fi
}

if [[ ${1:-} == serial ]]; then
  export ROS_DOMAIN_ID=28 ROS_LOCALHOST_ONLY=1
  namespace="/serial_test_$$"
  uart=${2:-}
  if [[ -z $uart ]]; then
    start_stm32_sim
  fi
  serial_prefix=$(ros2 pkg prefix communication_pkg)
  "$serial_prefix/lib/communication_pkg/serial_node" --ros-args -r "__ns:=$namespace" \
    -p "uart.device:=$uart" &
  serial_pid=$!
  trap cleanup EXIT
  ros2 run tests_pkg serial_test_node --ros-args -r "__ns:=$namespace"
  exit
fi
launch=robot.launch.py
if [[ ${1:-} == approach ]]; then
  launch=approach_test.launch.py
  shift
fi
for arg in "$@"; do
  if [[ $arg == simulate_uart:=true ]]; then
    export ROS_DOMAIN_ID=27
    start_stm32_sim
    ros2 launch robot_launch "$launch" "$@" "uart:=$uart"
    exit
  fi
done
exec ros2 launch robot_launch "$launch" "$@"

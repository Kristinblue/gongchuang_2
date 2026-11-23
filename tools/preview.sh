#!/usr/bin/env bash
set -euo pipefail

# 允许从任意目录运行，也可以通过 ~/.local/bin 下的软链接运行。
script=$(readlink -f -- "${BASH_SOURCE[0]}")
cd "$(dirname "$script")/.."
unit=gongchuang-preview

case "${1:-start}" in
  ensure-preview)
    if systemctl --user is-active --quiet "$unit.service"; then exit 0; fi
    exec bash "$script" start preview
    ;;
  start-task)
    # ROS Humble 的环境脚本会读取尚未设置的变量。
    set +u
    source /opt/ros/humble/setup.bash
    source install/local_setup.bash
    set -u
    exec timeout 8s ros2 service call /task/start std_srvs/srv/Trigger '{}'
    ;;
  start)
    if [[ ! -f install/local_setup.bash ]]; then
      echo "请先运行 bash "$PWD/tools/build.sh"" >&2
      exit 1
    fi
    mode=${2:-preview}
    launch_mode=$mode
    args=(source:=usb camera_fps:=10 camera_fourcc:=YUYV)
    ros_domain=${ROS_DOMAIN_ID:-0}
    case "$mode" in
      preview)
        args+=(safety_detector:=true)
        model="$PWD/assets/models/train5.rknn"
        if [[ -r "$model" ]]; then args+=("model:=$model"); fi
        ;;
      camera_test) ;;
      task)
        team=${3:-}
        if [[ $team != red && $team != blue ]]; then
          echo '请选择红方或蓝方' >&2
          exit 2
        fi
        model="$PWD/assets/models/train5.rknn"
        if [[ ! -r "$model" ]]; then
          echo "正式任务需要模型 $model" >&2
          exit 1
        fi
        launch_mode=preview
        args+=("model:=$model" uart:=/dev/ttyS6 "team:=$team" safety_detector:=true)
        ;;
      approach)
        model="$PWD/assets/models/train5.rknn"
        if [[ ! -r "$model" ]]; then
          echo "靠近测试需要模型 $model" >&2
          exit 1
        fi
        args+=("model:=$model" simulate_uart:=true safety_detector:=true)
        ;;
      *) echo "未知启动模式: $mode" >&2; exit 2 ;;
    esac

    description="Gongchuang $mode"
    if [[ $mode == task ]]; then description+=" $team"; fi
    if systemctl --user is-active --quiet "$unit.service"; then
      if [[ $(systemctl --user show "$unit.service" -p Description --value) == "$description" ]]; then
        echo "$mode 已在运行"
        exit 0
      fi
      systemctl --user stop "$unit.service"
    fi
    systemd-run --user --unit="$unit" --collect --description="$description" \
      --working-directory="$PWD" --setenv=OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT=2 \
      --setenv=ROS_DOMAIN_ID="$ros_domain" \
      --setenv=FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/tools/preview_fastdds.xml" \
      /bin/bash "$PWD/tools/run.sh" "mode:=$launch_mode" "${args[@]}"
    ;;
  stop)
    systemctl --user stop "$unit.service"
    ;;
  status)
    systemctl --user status "$unit.service" --no-pager
    ;;
  *)
    echo "用法: bash $script [start [preview|approach|camera_test|task red|task blue]|start-task|ensure-preview|stop|status]" >&2
    exit 2
    ;;
esac

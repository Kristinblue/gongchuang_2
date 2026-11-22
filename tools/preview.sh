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
  start)
    if [[ ! -f install/local_setup.bash ]]; then
      echo "请先运行 bash "$PWD/tools/build.sh"" >&2
      exit 1
    fi
    mode=${2:-preview}
    args=(source:=usb "mode:=$mode" camera_fps:=10 camera_fourcc:=YUYV)
    ros_domain=${ROS_DOMAIN_ID:-0}
    case "$mode" in
      preview)
        args+=(safety_detector:=true)
        model="$PWD/assets/models/train5.rknn"
        if [[ -r "$model" ]]; then args+=("model:=$model"); fi
        ;;
      camera_test) ;;
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
      /bin/bash "$PWD/tools/run.sh" "${args[@]}"
    ;;
  stop)
    systemctl --user stop "$unit.service"
    ;;
  status)
    systemctl --user status "$unit.service" --no-pager
    ;;
  *)
    echo "用法: bash $script [start [preview|approach|camera_test]|ensure-preview|stop|status]" >&2
    exit 2
    ;;
esac

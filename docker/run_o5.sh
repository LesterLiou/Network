#!/usr/bin/env bash
set -Eeuo pipefail

# =========================
# Defaults (可用環境變數覆蓋)
# =========================
IMAGE_NAME="${IMAGE_NAME:-flent_network}"
CONTAINER_NAME="${CONTAINER_NAME:-flent_network_container}"
MODE="${MODE:-bash}"                      # bash|rosbridge|flent-gui|flent-plot
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
DATA_DIR_DEFAULT="$(pwd)/.."

# 可開關硬體/顯示
USE_GPU="${USE_GPU:-auto}"                # auto|on|off
USE_USB="${USE_USB:-auto}"                # auto|on|off
USE_X="${USE_X:-auto}"                    # auto|on|off

# flent-plot 用：圖輸出位置與格式
PLOT_OUT_DIR="${PLOT_OUT_DIR:-$(pwd)/plots}"
PLOT_FORMAT="${PLOT_FORMAT:-png}"         # png|pdf|svg
PLOT_TYPE="${PLOT_TYPE:-totals}"          # flent --list-plots 可見
PLOT_INPUT="${PLOT_INPUT:-}"              # 指定 *.flent.gz (可留空用互動選)

# =========================
# Helpers
# =========================
die() { echo "[ERROR] $*" >&2; exit 1; }
info(){ echo "[INFO] $*"; }

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS] [MODE]

MODE:
  bash          進入容器 bash
  rosbridge     執行 /usr/local/bin/start_rosbridge.sh
  flent-gui     執行 flent --gui（需要可用的 X 顯示）
  flent-plot    用 CLI 產圖（不需要 GUI）

OPTIONS:
  -i, --image NAME          指定 image 名稱 (預設: ${IMAGE_NAME})
  -n, --name NAME           指定 container 名稱 (預設: ${CONTAINER_NAME})
  -d, --domain ID           設定 ROS_DOMAIN_ID (預設: ${ROS_DOMAIN_ID})
  -m, --mode MODE           同上 MODE 參數
  --data DIR                將專案/資料掛載到 /root/NETWORK (預設: ${DATA_DIR_DEFAULT})

  顯示/硬體：
  --gpu [auto|on|off]       啟用 GPU (nvidia) 傳遞 (預設: ${USE_GPU})
  --usb [auto|on|off]       掛載 USB (RealSense 等) (預設: ${USE_USB})
  --x   [auto|on|off]       啟用 X11 顯示 (預設: ${USE_X})

  flent-plot 參數：
  --plot-out DIR            圖檔輸出資料夾 (預設: ${PLOT_OUT_DIR})
  --plot-format FMT         png|pdf|svg (預設: ${PLOT_FORMAT})
  --plot-type TYPE          例如 totals、ping_cdf (預設: ${PLOT_TYPE})
  --plot-input FILE         指定 .flent.gz 檔 (可多個，以空白分隔；預設空)

  -h, --help                顯示說明

範例：
  $(basename "$0") -m bash
  $(basename "$0") --mode flent-gui
  $(basename "$0") --mode flent-plot --plot-type totals --plot-input ./output/*.flent.gz
EOF
}

# =========================
# 參數解析
# =========================
DATA_DIR="${DATA_DIR_DEFAULT}"
ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -i|--image) IMAGE_NAME="$2"; shift 2;;
    -n|--name) CONTAINER_NAME="$2"; shift 2;;
    -d|--domain) ROS_DOMAIN_ID="$2"; shift 2;;
    -m|--mode) MODE="$2"; shift 2;;
    --data) DATA_DIR="$2"; shift 2;;

    --gpu) USE_GPU="${2:-auto}"; shift 2;;
    --usb) USE_USB="${2:-auto}"; shift 2;;
    --x)   USE_X="${2:-auto}"; shift 2;;

    --plot-out) PLOT_OUT_DIR="$2"; shift 2;;
    --plot-format) PLOT_FORMAT="$2"; shift 2;;
    --plot-type) PLOT_TYPE="$2"; shift 2;;
    --plot-input) PLOT_INPUT="$2"; shift 2;;

    -h|--help) usage; exit 0;;
    bash|rosbridge|flent-gui|flent-plot) MODE="$1"; shift;;
    *) ARGS+=("$1"); shift;;
  esac
done

# =========================
# 前置檢查
# =========================
command -v docker >/dev/null 2>&1 || die "需要 docker"
[[ -d "${DATA_DIR}" ]] || die "--data 指定的資料夾不存在: ${DATA_DIR}"

# 若容器同名存在，先移除
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}\$"; then
  info "移除同名容器: ${CONTAINER_NAME}"
  docker rm -f "${CONTAINER_NAME}" >/dev/null || true
fi

# =========================
# GUI (X11) 偵測
# =========================
HAS_X=0
if [[ "${USE_X}" != "off" ]]; then
  if [[ -n "${DISPLAY-}" ]] && [[ -S /tmp/.X11-unix/X0 ]] && command -v xhost >/dev/null 2>&1; then
    # 有可用 X11
    xhost +local:root 1>/dev/null || true
    trap 'xhost -local:root 1>/dev/null || true' EXIT
    HAS_X=1
  fi
  # Wayland 多半不建議容器直通，除非自行設置 Xwayland，這裡就不自動了
fi

# =========================
# GPU 偵測 (NVIDIA)
# =========================
GPU_ARGS=()
if [[ "${USE_GPU}" != "off" ]]; then
  if [[ "${USE_GPU}" == "on" ]]; then
    GPU_ARGS+=(--gpus all)
  elif [[ "${USE_GPU}" == "auto" ]]; then
    if [[ -e /dev/nvidia0 ]] || command -v nvidia-smi >/dev/null 2>&1; then
      GPU_ARGS+=(--gpus all)
    fi
  fi
fi

# =========================
# USB 偵測 (RealSense 等)
# =========================
USB_ARGS=()
if [[ "${USE_USB}" != "off" ]]; then
  if [[ -d /dev/bus/usb ]]; then
    USB_ARGS+=(--device /dev/bus/usb:/dev/bus/usb)
  fi
fi

# =========================
# 組合共通參數
# =========================
COMMON_ARGS=(
  --rm
  --name "${CONTAINER_NAME}"
  --hostname "${CONTAINER_NAME}"
  --net=host
  --ipc=host
  --ulimit core=-1
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
  -e ROS_LOCALHOST_ONLY=0
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  -v "${DATA_DIR}":/root/NETWORK
  --privileged
)
# 顯示
if [[ $HAS_X -eq 1 ]]; then
  COMMON_ARGS+=( -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix )
fi
# GPU/USB
COMMON_ARGS+=( "${GPU_ARGS[@]}" "${USB_ARGS[@]}" )

info "Image         : ${IMAGE_NAME}"
info "Container     : ${CONTAINER_NAME}"
info "Mode          : ${MODE}"
info "ROS_DOMAIN_ID : ${ROS_DOMAIN_ID}"
info "X11           : $([[ $HAS_X -eq 1 ]] && echo on || echo off)"
info "GPU args      : ${GPU_ARGS[*]:-(none)}"
info "USB args      : ${USB_ARGS[*]:-(none)}"
info "Data mount    : ${DATA_DIR} -> /root/NETWORK"

# =========================
# 執行模式
# =========================
case "${MODE}" in
  bash)
    exec docker run -it "${COMMON_ARGS[@]}" "${IMAGE_NAME}" bash
    ;;

  rosbridge)
    exec docker run -it "${COMMON_ARGS[@]}" "${IMAGE_NAME}" /usr/local/bin/start_rosbridge.sh "${ARGS[@]}"
    ;;

  flent-gui)
    if [[ $HAS_X -ne 1 ]]; then
      die "目前無法偵測可用的 X 顯示環境（DISPLAY/xhost）。可改用 'flent-plot' 模式輸出圖檔。"
    fi
    exec docker run -it "${COMMON_ARGS[@]}" "${IMAGE_NAME}" bash -lc "flent --gui ${ARGS[*]:-}"
    ;;

  flent-plot)
    mkdir -p "${PLOT_OUT_DIR}"
    # 若沒指定輸入，嘗試在 /root/NETWORK/output 找
    FLENT_IN=()
    if [[ -n "${PLOT_INPUT}" ]]; then
      # 允許多個，以空白分隔
      read -r -a FLENT_IN <<< "${PLOT_INPUT}"
    else
      # 預設在 output 底下找
      FLENT_IN=(/root/NETWORK/output/*.flent.gz)
    fi
    info "Plot type     : ${PLOT_TYPE}"
    info "Plot format   : ${PLOT_FORMAT}"
    info "Plot out dir  : ${PLOT_OUT_DIR}"
    info "Plot inputs   : ${FLENT_IN[*]:-(none found)}"

    exec docker run -it "${COMMON_ARGS[@]}" -v "${PLOT_OUT_DIR}":/root/plots "${IMAGE_NAME}" \
      bash -lc "flent -p '${PLOT_TYPE}' --figure-format='${PLOT_FORMAT}' --figure-dir=/root/plots ${FLENT_IN[*]:-} && ls -l /root/plots"
    ;;

  *)
    usage
    exit 1;;
esac

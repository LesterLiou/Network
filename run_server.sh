#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# -------- CONFIG --------
PORT_NETPERF=12865      #for flent netserver
PORT_IPERF_MAIN=5201    #for TCP
PORT_IPERF_BG=5002      #for UDP
LOGDIR="/var/log/network_services"
mkdir -p "$LOGDIR"

# -------- LOG UTILS --------
log()   { echo "[INFO] $*" | tee -a "$LOGDIR/$(basename "$0").log"; }
warn()  { echo "[WARN] $*" | tee -a "$LOGDIR/$(basename "$0").log"; }
error() { echo "[ERROR] $*" | tee -a "$LOGDIR/$(basename "$0").log"; exit 1; }

# -------- FUNCTION --------
check_cmd() {
  command -v "$1" >/dev/null 2>&1 || error "❌ Missing command '$1'. Please install it before running this script."
}

start_server() {
  local name="$1"
  local port="$2"
  local cmd="$3"
  local logfile="${LOGDIR}/${name}_${port}.log"

  if ss -tuln | grep -q ":${port} "; then
    log "$name is already running (port $port)"
  else
    log "Starting $name on port $port..."
    eval "nohup $cmd -p $port > \"$logfile\" 2>&1 &"
    sleep 1

    # Verify if it started successfully
    if ss -tuln | grep -q ":${port} "; then
      log "✅ $name started successfully (port $port) → log: $logfile"
    else
      error "❌ Failed to start $name (port $port), check log: $logfile"
    fi
  fi
}

# -------- KILL MODE (--kill) --------
if [[ "${1:-}" == "--kill" ]]; then
  log "🛑 Stopping all iperf3 / netserver processes..."
  pkill -f "iperf3 -s" || true
  pkill -f "netserver" || true
  log "✅ All services stopped"
  exit 0
fi

# -------- MAIN --------
log " Checking required commands..."
for bin in ss netserver iperf3; do check_cmd "$bin"; done

log " Starting network servers..."

start_server "netserver" "$PORT_NETPERF" "netserver"
start_server "iperf3_main" "$PORT_IPERF_MAIN" "iperf3 -s"
start_server "iperf3_bg" "$PORT_IPERF_BG" "iperf3 -s"

log " Currently listening services:"
ss -tuln | grep -E ":(${PORT_NETPERF}|${PORT_IPERF_MAIN}|${PORT_IPERF_BG})" || warn "⚠️ No services found listening?"

log " Currently running processes:"
ps aux | grep -E "iperf3 -s|netserver" | grep -v grep || warn "⚠️ No related processes found?"

log "✅ All network services are up and running. You can now start testing with Flent or iperf3."


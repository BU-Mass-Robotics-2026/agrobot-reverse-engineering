#!/usr/bin/env bash
set -eo pipefail

ENV_FILE="${HOME}/agrobot_ws/scripts/can_bootstrap.env"
if [ -f "$ENV_FILE" ]; then
  source "$ENV_FILE"
fi

CAN_IFACE="${CAN_IFACE:-can0}"
CAN_SERIAL_DEV="${CAN_SERIAL_DEV:-/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_205236944B34-if00}"
SLCAN_SPEED_CODE="${SLCAN_SPEED_CODE:-8}"

log() {
  echo "[ensure_can_interface] $*"
}

have_iface() {
  ip link show "$CAN_IFACE" >/dev/null 2>&1
}

slcand_running() {
  pgrep -fa "slcand.* ${CAN_IFACE}$" >/dev/null 2>&1
}

main() {
  log "checking for $CAN_IFACE"

  sudo modprobe can >/dev/null 2>&1 || true
  sudo modprobe can_raw >/dev/null 2>&1 || true
  sudo modprobe slcan >/dev/null 2>&1 || true

  if have_iface; then
    log "$CAN_IFACE already exists; bringing it up"
    sudo ip link set "$CAN_IFACE" up || true
    log "$CAN_IFACE is ready"
    exit 0
  fi

  if [ ! -e "$CAN_SERIAL_DEV" ]; then
    log "FAILED: serial CAN adapter not found at $CAN_SERIAL_DEV"
    exit 1
  fi

  if slcand_running; then
    log "slcand already running for $CAN_IFACE"
  else
    log "attempting slcand bind from $CAN_SERIAL_DEV -> $CAN_IFACE (s${SLCAN_SPEED_CODE})"
    sudo slcand -o -c -f -s"${SLCAN_SPEED_CODE}" "$CAN_SERIAL_DEV" "$CAN_IFACE"
    sleep 1
  fi

  if ! have_iface; then
    log "FAILED: slcand did not create $CAN_IFACE"
    exit 1
  fi

  sudo ip link set "$CAN_IFACE" up
  log "$CAN_IFACE created successfully through slcand"
}

main "$@"

#!/usr/bin/env bash
set -euo pipefail

# ---- Config ---------------------------------------------------------------
NODE_NS="${NODE_NS:-/node_1}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WS_SETUP="${WS_SETUP:-$HOME/agrobot_ws/install/setup.bash}"

# EPOS2 object dictionary entries used here
IDX_CONTROLWORD=24640   # 0x6040
IDX_STATUSWORD=24641    # 0x6041
IDX_OPMODE=24672        # 0x6060
IDX_OPMODE_DISPLAY=24673# 0x6061
IDX_POS_ACTUAL=24676    # 0x6064
IDX_VEL_ACTUAL=24684    # 0x606C
IDX_TARGET_POS=24698    # 0x607A
IDX_MAX_PROFILE_VEL=24703 # 0x607F
IDX_PROFILE_VEL=24705     # 0x6081
IDX_PROFILE_ACCEL=24707   # 0x6083
IDX_PROFILE_DECEL=24708   # 0x6084
IDX_QUICKSTOP_DECEL=24709 # 0x6085
IDX_PROFILE_TYPE=24710    # 0x6086

# Controlword values
CW_SHUTDOWN=6        # 0x0006
CW_ENABLE=15         # 0x000F
CW_START_ABS=63      # 0x003F  absolute move, start immediately

# ---- ROS env --------------------------------------------------------------
if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
fi

if [[ -f "$WS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$WS_SETUP"
fi

# ---- Helpers --------------------------------------------------------------
log() {
  echo "[epos2_ppm] $*"
}

die() {
  echo "[epos2_ppm] ERROR: $*" >&2
  exit 1
}

require_ros2() {
  command -v ros2 >/dev/null 2>&1 || die "ros2 not found in PATH"
}

sdo_read() {
  local index="$1"
  local subindex="${2:-0}"
  local out
  out="$(ros2 service call "${NODE_NS}/sdo_read" canopen_interfaces/srv/CORead \
    "{index: ${index}, subindex: ${subindex}}" 2>&1 || true)"

  if ! grep -q "success=True" <<<"$out"; then
    echo "$out" >&2
    return 1
  fi

  sed -n 's/.*data=\([0-9]\+\).*/\1/p' <<<"$out" | tail -n1
}

sdo_write() {
  local index="$1"
  local subindex="${2:-0}"
  local data="$3"
  local out
  out="$(ros2 service call "${NODE_NS}/sdo_write" canopen_interfaces/srv/COWrite \
    "{index: ${index}, subindex: ${subindex}, data: ${data}}" 2>&1 || true)"

  if ! grep -q "success=True" <<<"$out"; then
    echo "$out" >&2
    return 1
  fi
}

print_u32() {
  local label="$1"
  local val="$2"
  printf "%-18s %10u (0x%X)\n" "$label" "$val" "$val"
}

status() {
  local sw mode pos vel
  sw="$(sdo_read "$IDX_STATUSWORD" 0)" || die "Failed reading statusword"
  mode="$(sdo_read "$IDX_OPMODE_DISPLAY" 0)" || die "Failed reading op mode display"
  pos="$(sdo_read "$IDX_POS_ACTUAL" 0)" || die "Failed reading actual position"
  vel="$(sdo_read "$IDX_VEL_ACTUAL" 0)" || die "Failed reading actual velocity"

  print_u32 "Statusword" "$sw"
  print_u32 "Mode display" "$mode"
  print_u32 "Position" "$pos"
  print_u32 "Velocity" "$vel"
}

enable_ppm() {
  log "Setting Profile Position Mode (0x6060 = 1)"
  sdo_write "$IDX_OPMODE" 0 1 || die "Failed writing op mode"

  log "Sending Shutdown (0x6040 = 0x0006)"
  sdo_write "$IDX_CONTROLWORD" 0 "$CW_SHUTDOWN" || die "Failed writing shutdown controlword"
  sleep 0.2

  log "Sending Enable Operation (0x6040 = 0x000F)"
  sdo_write "$IDX_CONTROLWORD" 0 "$CW_ENABLE" || die "Failed writing enable controlword"
  sleep 0.2

  log "Current state:"
  status
}

set_profile_params() {
  local maxvel="$1"
  local vel="$2"
  local accel="$3"
  local decel="$4"
  local quickstop="${5:-$decel}"
  local profile_type="${6:-0}"   # 0 = trapezoidal, 1 = sin²

  log "Writing motion profile parameters"
  sdo_write "$IDX_MAX_PROFILE_VEL" 0 "$maxvel" || die "Failed writing max profile velocity"
  sdo_write "$IDX_PROFILE_VEL" 0 "$vel" || die "Failed writing profile velocity"
  sdo_write "$IDX_PROFILE_ACCEL" 0 "$accel" || die "Failed writing profile acceleration"
  sdo_write "$IDX_PROFILE_DECEL" 0 "$decel" || die "Failed writing profile deceleration"
  sdo_write "$IDX_QUICKSTOP_DECEL" 0 "$quickstop" || die "Failed writing quickstop deceleration"
  sdo_write "$IDX_PROFILE_TYPE" 0 "$profile_type" || die "Failed writing profile type"

  log "Profile params set."
  print_u32 "Max profile vel" "$maxvel"
  print_u32 "Profile vel" "$vel"
  print_u32 "Profile accel" "$accel"
  print_u32 "Profile decel" "$decel"
  print_u32 "Quickstop decel" "$quickstop"
  print_u32 "Profile type" "$profile_type"
}

move_abs() {
  local target="$1"

  log "Writing target position (0x607A) = $target"
  sdo_write "$IDX_TARGET_POS" 0 "$target" || die "Failed writing target position"

  log "Starting absolute move immediately (0x6040 = 0x003F)"
  sdo_write "$IDX_CONTROLWORD" 0 "$CW_START_ABS" || die "Failed writing start controlword"

  sleep 0.1

  # Return to normal enabled controlword so the next start edge is clean.
  sdo_write "$IDX_CONTROLWORD" 0 "$CW_ENABLE" || die "Failed restoring enabled controlword"
}

move_rel() {
  local delta="$1"
  local cur target
  cur="$(sdo_read "$IDX_POS_ACTUAL" 0)" || die "Failed reading current position"
  target=$((cur + delta))
  log "Current position = $cur ; relative delta = $delta ; target = $target"
  move_abs "$target"
}

usage() {
  cat <<EOF
Usage:
  $0 status
  $0 enable
  $0 params <max_vel> <vel> <accel> <decel> [quickstop_decel] [profile_type]
  $0 move_abs <target_qc>
  $0 move_rel <delta_qc>

Examples:
  $0 status
  $0 enable
  $0 params 5000 1000 2000 2000
  $0 move_abs 1000
  $0 move_rel 5000

Notes:
  - This is pure Profile Position Mode (0x6060 = 1).
  - Units for 0x607A / 0x6064 are drive position counts (qc).
  - Run your driver-only launch first so /node_1/sdo_read and /node_1/sdo_write exist.
  - Override node namespace if needed:
      NODE_NS=/node_1 $0 status
EOF
}

# ---- Main ----------------------------------------------------------------
require_ros2

cmd="${1:-}"
case "$cmd" in
  status)
    status
    ;;
  enable)
    enable_ppm
    ;;
  params)
    [[ $# -ge 5 ]] || die "params needs at least 4 numeric args"
    set_profile_params "$2" "$3" "$4" "$5" "${6:-$5}" "${7:-0}"
    ;;
  move_abs)
    [[ $# -eq 2 ]] || die "move_abs needs 1 arg"
    move_abs "$2"
    ;;
  move_rel)
    [[ $# -eq 2 ]] || die "move_rel needs 1 arg"
    move_rel "$2"
    ;;
  *)
    usage
    exit 1
    ;;
esac
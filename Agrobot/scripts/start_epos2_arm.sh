#!/usr/bin/env bash
set -eo pipefail

cd ~/agrobot_ws
source /opt/ros/jazzy/setup.bash
source ~/agrobot_ws/install/setup.bash

cleanup() {
  echo
  echo "[start_epos2_arm] stopping bridge/driver..."
  if [ -n "${BRIDGE_PID:-}" ]; then kill "${BRIDGE_PID}" 2>/dev/null || true; fi
  if [ -n "${DRIVER_PID:-}" ]; then kill "${DRIVER_PID}" 2>/dev/null || true; fi
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[start_epos2_arm] ensuring CAN interface exists..."
~/agrobot_ws/scripts/ensure_can_interface.sh

if ! ip link show can0 >/dev/null 2>&1; then
  echo "[start_epos2_arm] ERROR: can0 still does not exist after bootstrap"
  exit 1
fi

echo "[start_epos2_arm] CAN interface status:"
ip -details link show can0 || true

echo "[start_epos2_arm] starting CANopen driver..."
ros2 launch moveit_config j3_canopen_driver_only_ipm.launch.py &
DRIVER_PID=$!

echo "[start_epos2_arm] waiting for driver startup..."
sleep 6

if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
  echo "[start_epos2_arm] ERROR: driver exited during startup"
  exit 1
fi

echo "[start_epos2_arm] applying working live PDO remap for node 1..."
~/agrobot_ws/scripts/apply_ipm_pdo_remap_one.sh 1

echo "[start_epos2_arm] waiting after remap..."
sleep 3

if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
  echo "[start_epos2_arm] ERROR: driver died before bridge startup"
  exit 1
fi

echo "[start_epos2_arm] starting bridge..."
ros2 run epos2_bridge epos2_j3_bridge --ros-args \
  -p drive_node_id:=1 \
  -p joint_name:=joint3 \
  -p force_ipm_on_startup:=false \
  -p enable_on_startup:=false \
  -p fault_clear_on_startup:=false \
  -p encoder_qc_per_motor_rev:=25600.0 \
  -p gear_ratio_motor_per_joint_rev:=1.0 \
  -p sign:=1.0 \
  -p ipm_default_segment_ms:=100 \
  --ros-args -r /joint_states:=/epos2/j3/joint_states_raw &
BRIDGE_PID=$!

echo "[start_epos2_arm] system running"
wait

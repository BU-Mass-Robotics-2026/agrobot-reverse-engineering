#!/usr/bin/env bash
set -eo pipefail

cd ~/agrobot_ws
source /opt/ros/jazzy/setup.bash
source ~/agrobot_ws/install/setup.bash

cleanup() {
  echo
  echo "[start_full_motion_stack] stopping processes..."
  if [ -n "${MOVEIT_PID:-}" ]; then kill "${MOVEIT_PID}" 2>/dev/null || true; fi
  if [ -n "${ARM_CTRL_PID:-}" ]; then kill "${ARM_CTRL_PID}" 2>/dev/null || true; fi
  if [ -n "${MERGE_PID:-}" ]; then kill "${MERGE_PID}" 2>/dev/null || true; fi
  if [ -n "${EPOS_PID:-}" ]; then kill "${EPOS_PID}" 2>/dev/null || true; fi
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[start_full_motion_stack] starting hardware stack..."
~/agrobot_ws/scripts/start_epos2_arm.sh &
EPOS_PID=$!

echo "[start_full_motion_stack] waiting for hardware stack..."
sleep 8

if ! kill -0 "$EPOS_PID" 2>/dev/null; then
  echo "[start_full_motion_stack] ERROR: hardware stack exited during startup"
  exit 1
fi

echo "[start_full_motion_stack] running J3 preflight..."
~/agrobot_ws/scripts/j3_preflight.sh

echo "[start_full_motion_stack] starting joint-state merger..."
python3 ~/agrobot_ws/scripts/merge_joint_states.py &
MERGE_PID=$!

sleep 2

echo "[start_full_motion_stack] starting arm controller..."
ros2 launch epos2_bridge epos2_arm_controller.launch.py &
ARM_CTRL_PID=$!

sleep 4

echo "[start_full_motion_stack] starting MoveIt / RViz..."
ros2 launch moveit_config demo.launch.py &
MOVEIT_PID=$!

echo "[start_full_motion_stack] full stack running"
wait

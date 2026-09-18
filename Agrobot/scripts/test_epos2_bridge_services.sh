#!/usr/bin/env bash
set -eo pipefail

cd ~/agrobot_ws
source /opt/ros/jazzy/setup.bash
source ~/agrobot_ws/install/setup.bash

echo "[test] waiting for bridge services..."
for i in $(seq 1 20); do
  if ros2 service list | grep -q /epos2/j3/arm_ipm; then
    break
  fi
  sleep 1
done

echo "[test] service list:"
ros2 service list | grep /epos2/j3 || true

echo "[test] arm"
ros2 service call /epos2/j3/arm_ipm std_srvs/srv/Trigger "{}"
sleep 1

echo "[test] move +0.02"
ros2 service call /epos2/j3/move_delta epos2_bridge_interfaces/srv/MoveDelta "{delta_rad: 0.02}"
sleep 1

echo "[test] move -0.02"
ros2 service call /epos2/j3/move_delta epos2_bridge_interfaces/srv/MoveDelta "{delta_rad: -0.02}"
sleep 1

echo "[test] disarm"
ros2 service call /epos2/j3/disarm_ipm std_srvs/srv/Trigger "{}"
sleep 1

echo "[test] refusal after disarm"
ros2 service call /epos2/j3/move_delta epos2_bridge_interfaces/srv/MoveDelta "{delta_rad: 0.02}"

#!/usr/bin/env bash
set -eo pipefail

cd ~/agrobot_ws
source /opt/ros/jazzy/setup.bash
source ~/agrobot_ws/install/setup.bash

fail() {
  echo "[j3_preflight] FAIL: $*"
  exit 1
}

echo "[j3_preflight] services:"
ros2 service list | grep /epos2/j3 || true

echo
echo "[j3_preflight] pre-arm state reads"
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 24641, subindex: 0}" || true   # 0x6041
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 24673, subindex: 0}" || true   # 0x6061
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 8388,  subindex: 1}" || true   # 0x20C4:01
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 4099,  subindex: 0}" || true   # 0x1003:00

echo
echo "[j3_preflight] clear fault"
CF_OUT="$(ros2 service call /epos2/j3/clear_fault std_srvs/srv/Trigger "{}" 2>&1 || true)"
echo "$CF_OUT"
echo "$CF_OUT" | grep -q "success=True" || fail "clear_fault did not succeed"

echo
echo "[j3_preflight] arm ipm"
ARM_OUT="$(ros2 service call /epos2/j3/arm_ipm std_srvs/srv/Trigger "{}" 2>&1 || true)"
echo "$ARM_OUT"
echo "$ARM_OUT" | grep -q "success=True" || fail "arm_ipm did not succeed"

echo
echo "[j3_preflight] post-arm state reads"
POST_SW="$(ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 24641, subindex: 0}" 2>&1 || true)"
POST_MD="$(ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 24673, subindex: 0}" 2>&1 || true)"
POST_IPM="$(ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 8388, subindex: 1}" 2>&1 || true)"
echo "$POST_SW"
echo "$POST_MD"
echo "$POST_IPM"

echo
echo "[j3_preflight] disarm ipm"
DISARM_OUT="$(ros2 service call /epos2/j3/disarm_ipm std_srvs/srv/Trigger "{}" 2>&1 || true)"
echo "$DISARM_OUT"
echo "$DISARM_OUT" | grep -q "success=True" || fail "disarm_ipm did not succeed"

echo "[j3_preflight] PASS"

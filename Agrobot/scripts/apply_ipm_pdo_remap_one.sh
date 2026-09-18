#!/usr/bin/env bash
set -eo pipefail

NODE_ID="${1:?usage: $0 NODE_ID}"

cd ~/agrobot_ws
source /opt/ros/jazzy/setup.bash
source ~/agrobot_ws/install/setup.bash

write() {
  local idx="$1"
  local sub="$2"
  local data="$3"
  ros2 service call /node_${NODE_ID}/sdo_write canopen_interfaces/srv/COWrite "{index: ${idx}, subindex: ${sub}, data: ${data}}"
}

RPDO1=$((0x200 + NODE_ID))
RPDO2=$((0x300 + NODE_ID))
TPDO1=$((0x180 + NODE_ID))
TPDO2=$((0x280 + NODE_ID))

RPDO1_DISABLE=$((0x80000000 + RPDO1))
RPDO2_DISABLE=$((0x80000000 + RPDO2))
TPDO1_DISABLE=$((0xC0000000 + TPDO1))
TPDO2_DISABLE=$((0xC0000000 + TPDO2))

echo "Node ${NODE_ID}: entering Pre-Operational"
cansend can0 000#80$(printf "%02X" ${NODE_ID})

echo "Node ${NODE_ID}: disabling PDOs"
write 5120 1 "${RPDO1_DISABLE}"
write 5121 1 "${RPDO2_DISABLE}"
write 6144 1 "${TPDO1_DISABLE}"
write 6145 1 "${TPDO2_DISABLE}"

echo "Node ${NODE_ID}: mapping RPDO1 = 0x20C1"
write 5632 0 0
write 5632 1 549519424      # 0x20C10040
write 5632 0 1

echo "Node ${NODE_ID}: mapping RPDO2 = 0x6040 + 0x6060"
write 5633 0 0
write 5633 1 1614807056     # 0x60400010
write 5633 2 1616904200     # 0x60600008
write 5633 0 2

echo "Node ${NODE_ID}: mapping TPDO1 = 0x20C4:01 + 0x6041 + 0x6061"
write 6656 0 0
write 6656 1 549716240      # 0x20C40110
write 6656 2 1614872592     # 0x60410010
write 6656 3 1616969736     # 0x60610008
write 6656 0 3

echo "Node ${NODE_ID}: mapping TPDO2 = 0x6064 + 0x606C"
write 6657 0 0
write 6657 1 1617166368     # 0x60640020
write 6657 2 1617690656     # 0x606C0020
write 6657 0 2

echo "Node ${NODE_ID}: re-enabling PDOs"
write 5120 1 "${RPDO1}"
write 5120 2 255
write 5121 1 "${RPDO2}"
write 5121 2 255
write 6144 1 $((0x40000000 + TPDO1))
write 6144 2 255
write 6145 1 $((0x40000000 + TPDO2))
write 6145 2 255

echo "Node ${NODE_ID}: back to Operational"
cansend can0 000#01$(printf "%02X" ${NODE_ID})

echo "Node ${NODE_ID}: done"

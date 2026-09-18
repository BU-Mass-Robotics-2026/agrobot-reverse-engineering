#!/usr/bin/env bash
set -e

cd ~/agrobot_ws
source /opt/ros/jazzy/setup.bash
source ~/agrobot_ws/install/setup.bash

echo "Putting node 1 into Pre-Operational..."
cansend can0 000#8001

echo "Disabling PDOs..."
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5120, subindex: 1, data: 2147484161}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5121, subindex: 1, data: 2147484417}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6144, subindex: 1, data: 3221225857}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6145, subindex: 1, data: 3221226113}"

echo "Writing RPDO1 = 0x20C1..."
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5632, subindex: 0, data: 0}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5632, subindex: 1, data: 549519424}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5632, subindex: 0, data: 1}"

echo "Writing RPDO2 = 0x6040 + 0x6060..."
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5633, subindex: 0, data: 0}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5633, subindex: 1, data: 1614807056}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5633, subindex: 2, data: 1616904200}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5633, subindex: 0, data: 2}"

echo "Writing TPDO1 = 0x20C4:01 + 0x6041 + 0x6061..."
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6656, subindex: 0, data: 0}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6656, subindex: 1, data: 549716240}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6656, subindex: 2, data: 1614872592}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6656, subindex: 3, data: 1616969736}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6656, subindex: 0, data: 3}"

echo "Writing TPDO2 = 0x6064 + 0x606C..."
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6657, subindex: 0, data: 0}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6657, subindex: 1, data: 1617166368}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6657, subindex: 2, data: 1617690656}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6657, subindex: 0, data: 2}"

echo "Re-enabling PDOs..."
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5120, subindex: 1, data: 513}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5120, subindex: 2, data: 255}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5121, subindex: 1, data: 769}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 5121, subindex: 2, data: 255}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6144, subindex: 1, data: 1073742209}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6144, subindex: 2, data: 255}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6145, subindex: 1, data: 1073742465}"
ros2 service call /node_1/sdo_write canopen_interfaces/srv/COWrite "{index: 6145, subindex: 2, data: 255}"

echo "Returning node 1 to Operational..."
cansend can0 000#0101

echo "Done."

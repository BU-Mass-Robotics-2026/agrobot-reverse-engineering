#!/usr/bin/env bash
set -eo pipefail

cd ~/agrobot_ws
source /opt/ros/jazzy/setup.bash
source ~/agrobot_ws/install/setup.bash

echo "Checking live PDO map for node 1..."

ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 5632, subindex: 0}"
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 5632, subindex: 1}"
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 5633, subindex: 0}"
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 5633, subindex: 1}"
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 5633, subindex: 2}"

ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 6656, subindex: 0}"
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 6656, subindex: 1}"
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 6656, subindex: 2}"
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 6656, subindex: 3}"

ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 6657, subindex: 0}"
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 6657, subindex: 1}"
ros2 service call /node_1/sdo_read canopen_interfaces/srv/CORead "{index: 6657, subindex: 2}"

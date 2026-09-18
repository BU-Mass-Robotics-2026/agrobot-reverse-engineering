#!/usr/bin/env bash
set -euo pipefail

WS=~/agrobot_ws
PKG=$WS/src/epos2_bridge
LAUNCH_DIR=$PKG/launch
SCRIPTS_DIR=$WS/scripts

mkdir -p "$LAUNCH_DIR" "$SCRIPTS_DIR"

# -----------------------------
# Generic one-node remap script
# -----------------------------
cat > "$SCRIPTS_DIR/apply_ipm_pdo_remap_one.sh" << 'EOS'
#!/usr/bin/env bash
set -euo pipefail

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
EOS
chmod +x "$SCRIPTS_DIR/apply_ipm_pdo_remap_one.sh"

# --------------------------------
# Multi-node remap helper
# --------------------------------
cat > "$SCRIPTS_DIR/apply_ipm_pdo_remap_all.sh" << 'EOS'
#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -eq 0 ]; then
  echo "usage: $0 NODE_ID [NODE_ID ...]"
  echo "example: $0 1 2 3"
  exit 1
fi

for node_id in "$@"; do
  ~/agrobot_ws/scripts/apply_ipm_pdo_remap_one.sh "$node_id"
done
EOS
chmod +x "$SCRIPTS_DIR/apply_ipm_pdo_remap_all.sh"

python3 - << 'PY'
from pathlib import Path

launch_dir = Path("/home/robotics-club/agrobot_ws/src/epos2_bridge/launch")

# Defaults:
# - j3 preserves your current known-good setup on drive_node_id=1
# - all other node IDs/ratios are placeholders for you to edit
joint_defs = [
    ("j1", "joint_1", 2, 25600.0, 100.0, 1.0, 0.0),
    ("j2", "joint_2", 3, 25600.0, 100.0, 1.0, 0.0),
    ("j3", "joint_3", 1, 25600.0,   1.0, 1.0, 0.0),
    ("j4", "joint_4", 4, 25600.0, 100.0, 1.0, 0.0),
    ("j5", "joint_5", 5, 25600.0, 100.0, 1.0, 0.0),
    ("j6", "joint_6", 6, 25600.0, 100.0, 1.0, 0.0),
]

for short, joint_name, node_id, enc, gear, sign, zero in joint_defs:
    text = f"""from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="epos2_bridge",
            executable="epos2_j3_bridge",
            name="epos2_{short}_bridge",
            output="screen",
            parameters=[
                {{"drive_node_id": {node_id}}},
                {{"joint_name": "{joint_name}"}},
                {{"encoder_qc_per_motor_rev": {enc}}},
                {{"gear_ratio_motor_per_joint_rev": {gear}}},
                {{"sign": {sign}}},
                {{"zero_offset_qc": {zero}}},
                {{"force_ipm_on_startup": False}},
                {{"enable_on_startup": False}},
                {{"fault_clear_on_startup": False}},
                {{"ipm_default_segment_ms": 100}},
            ],
            remappings=[
                ("/epos2/j3/clear_fault", "/epos2/{short}/clear_fault"),
                ("/epos2/j3/arm_ipm", "/epos2/{short}/arm_ipm"),
                ("/epos2/j3/disarm_ipm", "/epos2/{short}/disarm_ipm"),
                ("/epos2/j3/move_delta", "/epos2/{short}/move_delta"),
                ("/epos2/j3/arm_ipm_now", "/epos2/{short}/arm_ipm_now"),
                ("/epos2/j3/disarm_ipm_now", "/epos2/{short}/disarm_ipm_now"),
                ("/epos2/j3/test_move_rad", "/epos2/{short}/test_move_rad"),
                ("/epos2/j3/joint_target", "/epos2/{short}/joint_target"),
                ("/epos2/j3/state_summary", "/epos2/{short}/state_summary"),
                ("/epos2/j3/state_raw", "/epos2/{short}/state_raw"),
                ("/epos2/j3/state_engineering", "/epos2/{short}/state_engineering"),
                ("/epos2/j3/fault", "/epos2/{short}/fault"),
            ],
        )
    ])
"""
    (launch_dir / f"epos2_{short}_bridge.launch.py").write_text(text)

arm_text = """from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    epos2_share = get_package_share_directory("epos2_bridge")
    moveit_config_share = get_package_share_directory("moveit_config")

    enable_j1 = LaunchConfiguration("enable_j1")
    enable_j2 = LaunchConfiguration("enable_j2")
    enable_j3 = LaunchConfiguration("enable_j3")
    enable_j4 = LaunchConfiguration("enable_j4")
    enable_j5 = LaunchConfiguration("enable_j5")
    enable_j6 = LaunchConfiguration("enable_j6")
    apply_live_remap = LaunchConfiguration("apply_live_remap")
    remap_node_ids = LaunchConfiguration("remap_node_ids")

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_share, "launch", "j3_canopen_driver_only_ipm.launch.py")
        )
    )

    remap_proc = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            ["source /opt/ros/jazzy/setup.bash && "
             "source ~/agrobot_ws/install/setup.bash && "
             "~/agrobot_ws/scripts/apply_ipm_pdo_remap_all.sh ", remap_node_ids]
        ],
        output="screen",
        condition=IfCondition(apply_live_remap),
    )

    def inc(name, cond):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(epos2_share, "launch", f"epos2_{name}_bridge.launch.py")),
            condition=IfCondition(cond),
        )

    return LaunchDescription([
        DeclareLaunchArgument("enable_j1", default_value="false"),
        DeclareLaunchArgument("enable_j2", default_value="false"),
        DeclareLaunchArgument("enable_j3", default_value="true"),
        DeclareLaunchArgument("enable_j4", default_value="false"),
        DeclareLaunchArgument("enable_j5", default_value="false"),
        DeclareLaunchArgument("enable_j6", default_value="false"),
        DeclareLaunchArgument("apply_live_remap", default_value="true"),
        DeclareLaunchArgument("remap_node_ids", default_value="1"),

        driver_launch,
        TimerAction(period=3.0, actions=[remap_proc]),
        TimerAction(period=6.0, actions=[inc("j1", enable_j1)]),
        TimerAction(period=6.5, actions=[inc("j2", enable_j2)]),
        TimerAction(period=7.0, actions=[inc("j3", enable_j3)]),
        TimerAction(period=7.5, actions=[inc("j4", enable_j4)]),
        TimerAction(period=8.0, actions=[inc("j5", enable_j5)]),
        TimerAction(period=8.5, actions=[inc("j6", enable_j6)]),
    ])
"""
(launch_dir / "epos2_arm.launch.py").write_text(arm_text)
PY

cat > "$SCRIPTS_DIR/start_epos2_arm.sh" << 'EOS'
#!/usr/bin/env bash
set -euo pipefail

cd ~/agrobot_ws
source /opt/ros/jazzy/setup.bash
source ~/agrobot_ws/install/setup.bash

# Current default: only j3 enabled, remap node ID 1.
ros2 launch epos2_bridge epos2_arm.launch.py \
  enable_j1:=false \
  enable_j2:=false \
  enable_j3:=true \
  enable_j4:=false \
  enable_j5:=false \
  enable_j6:=false \
  apply_live_remap:=true \
  remap_node_ids:="1"
EOS
chmod +x "$SCRIPTS_DIR/start_epos2_arm.sh"

echo
echo "Generated:"
echo "  - per-joint launch files in $LAUNCH_DIR"
echo "  - $SCRIPTS_DIR/apply_ipm_pdo_remap_one.sh"
echo "  - $SCRIPTS_DIR/apply_ipm_pdo_remap_all.sh"
echo "  - $SCRIPTS_DIR/start_epos2_arm.sh"
echo
echo "IMPORTANT:"
echo "  - edit the drive_node_id / encoder / gear / sign / zero values in each epos2_j*.launch.py"
echo "  - only enable joints that actually exist in moveit_config CANopen bus config"

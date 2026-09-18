from pathlib import Path
import sys

path = Path("/home/robotics-club/agrobot_ws/src/epos2_bridge/epos2_bridge/epos2_j3_bridge.py")
text = path.read_text()

def replace_once(old, new, label):
    global text
    if new in text:
        return
    if old not in text:
        raise RuntimeError(f"Could not find pattern for {label}")
    text = text.replace(old, new, 1)

replace_once(
    '        self.declare_parameter("can_interface", "can0")\n',
    '        self.declare_parameter("can_interface", "can0")\n'
    '        self.declare_parameter("drive_node_id", 1)\n',
    "drive_node_id parameter",
)

replace_once(
    '        self.kin = JointKinematics(\n',
    '        self.drive_node_id = int(self.get_parameter("drive_node_id").value)\n\n'
    '        self.kin = JointKinematics(\n',
    "drive_node_id assignment",
)

replace_once(
    '        self.read_cli = self.create_client(CORead, "/node_1/sdo_read", callback_group=self.cb_sdo)\n',
    '        self.read_cli = self.create_client(\n'
    '            CORead,\n'
    '            f"/node_{self.drive_node_id}/sdo_read",\n'
    '            callback_group=self.cb_sdo,\n'
    '        )\n',
    "dynamic read client",
)

replace_once(
    '        self.write_cli = self.create_client(COWrite, "/node_1/sdo_write", callback_group=self.cb_sdo)\n',
    '        self.write_cli = self.create_client(\n'
    '            COWrite,\n'
    '            f"/node_{self.drive_node_id}/sdo_write",\n'
    '            callback_group=self.cb_sdo,\n'
    '        )\n',
    "dynamic write client",
)

replace_once(
    '        self.can = RawSocketCAN(self.get_parameter("can_interface").value)\n',
    '        self.can = RawSocketCAN(self.get_parameter("can_interface").value)\n'
    '        self.cob_heartbeat = 0x700 + self.drive_node_id\n'
    '        self.cob_rpdo1 = 0x200 + self.drive_node_id\n'
    '        self.cob_rpdo2 = 0x300 + self.drive_node_id\n'
    '        self.cob_tpdo1 = 0x180 + self.drive_node_id\n'
    '        self.cob_tpdo2 = 0x280 + self.drive_node_id\n'
    '        self.cob_emcy = 0x080 + self.drive_node_id\n',
    "dynamic COB IDs",
)

replacements = {
    "self.can.send(COB_RPDO2, data)": "self.can.send(self.cob_rpdo2, data)",
    "self.can.send(COB_RPDO1, payload)": "self.can.send(self.cob_rpdo1, payload)",
    "if can_id == COB_HEARTBEAT and len(data) >= 1:": "if can_id == self.cob_heartbeat and len(data) >= 1:",
    "elif can_id == COB_TPDO1:": "elif can_id == self.cob_tpdo1:",
    "elif can_id == COB_TPDO2:": "elif can_id == self.cob_tpdo2:",
    "elif can_id == COB_EMCY and len(data) >= 3:": "elif can_id == self.cob_emcy and len(data) >= 3:",
}

for old, new in replacements.items():
    text = text.replace(old, new)

path.write_text(text)
print(f"Patched {path}")

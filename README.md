# AgroBot Reverse Engineering

Editable source copies for studying and modifying the BU Mass Robotics robot software.

## Projects

| Folder | Original repository | Imported branch |
| --- | --- | --- |
| [Agrobot](Agrobot/) | [BU-Mass-Robotics-2026/Agrobot](https://github.com/BU-Mass-Robotics-2026/Agrobot) | `main` |
| [EPOS2ROSBridge](EPOS2ROSBridge/) | [BU-Mass-Robotics-2026/EPOS2ROSBridge](https://github.com/BU-Mass-Robotics-2026/EPOS2ROSBridge) | `main` |
| [ros2_canopen](ros2_canopen/) | [BU-Mass-Robotics-2026/ros2_canopen](https://github.com/BU-Mass-Robotics-2026/ros2_canopen) | `master` |

All folders contain ordinary tracked files, not Git submodules. Changes committed and pushed here belong to this repository; they do not update the original repositories. Original project histories are available at the source links.

The nested CANopen dependencies in `Agrobot/src/ros2_canopen/` and `EPOS2ROSBridge/ros2_canopen/` are also editable copies, expanded at the exact version each original project referenced. They differ from the separately imported top-level `ros2_canopen/` version and are intentionally preserved separately.

See [SOURCES.md](SOURCES.md) for exact source commits and import verification. Original documentation and license files are preserved. This import does not establish that the software builds or runs; no robot software was executed.

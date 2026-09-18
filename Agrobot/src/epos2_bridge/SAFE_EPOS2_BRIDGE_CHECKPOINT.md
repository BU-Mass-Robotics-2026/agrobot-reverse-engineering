# Safe EPOS2 bridge checkpoint

This checkpoint implements the intended EPOS2 IPM production pattern:

- Legacy IPM hold-stream services are disabled by default with `allow_legacy_ipm_services := false`.
- FollowJointTrajectory is the production motion path.
- Sync-start waits before IPM activation.
- IPM is activated only after FIFO prefill.
- RPDO2 0x6040/0x6060 commands go through command_control_mode().
- Normal completion uses dt=0 terminator, clears bit 4 with 0x000F, then enters Maxon Position Mode 0xFF with 0x2062 hold target.
- Mid-stream cancel also attempts dt=0 termination and Position Mode hold.
- The old repeated stationary IPM hold-record pattern is quarantined.

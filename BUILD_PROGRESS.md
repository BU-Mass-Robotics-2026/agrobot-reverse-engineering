# Agrobot investigation progress

Goal: understand and document the existing stack, one authorized step at a time. Record what works, what fails, and missing dependencies in [INFRA.md](INFRA.md).

Do not change project code or configuration to make checks pass. Fixes require a separate request. A step is complete when its outcome is documented, even if it fails.

## Process flow

1. **Source baseline**
   - [x] Import the Agrobot, EPOS2ROSBridge, and ros2_canopen source trees with recorded revisions.
   - [x] Document the Agrobot workspace and overlapping source trees.
   - [x] Record observed dependency versions and the current OS/ROS baseline.

2. **Fresh laptop setup and build**
   - [x] Compile all 20 packages in `Agrobot/` on this laptop with ROS 2 Jazzy using isolated build directories.
   - [x] Run the newly prepared clean build.
   - [ ] Document dependency setup from a clean clone.
   - [ ] Attempt a fresh-environment build; record missing dependencies and failures.

3. **Robot launch and motion**
   - [x] Attempt existing launch files; record which components start or fail.
   - [ ] Send a mock-control command; record trajectory completion, joint feedback, and RViz behavior.
   - [ ] Document commands and results for repeating these checks.

4. **Existing software workflow**
   - [ ] Inspect picker/commander interfaces; document mismatches.
   - [ ] Assess existing picking and perception integration; note missing or unfinished parts.
   - [ ] Check cancellation, feedback, and startup/shutdown behavior; record failures.

5. **Hardware investigation, when authorized**
   - [ ] Compare documented configuration with the physical robot.
   - [ ] Record observed axis, gripper, and fault-handling behavior.
   - [ ] Document gaps preventing the existing picking workflow from operating.

## Current checkpoint

All 20 packages compiled. MoveIt, RViz, mock controllers, and commander started from the fresh install; startup findings are recorded in INFRA.md. Motion checks and tests against this install remain pending.

Build output is saved in [build_logs/compile-ybgPHn/terminal.log](build_logs/compile-ybgPHn/terminal.log). Runtime logs are under `build_logs/run-dgJ1XH/`. Paused after startup, with the mock stack running.

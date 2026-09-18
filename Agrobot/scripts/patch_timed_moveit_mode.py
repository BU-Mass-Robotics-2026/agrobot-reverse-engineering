#!/usr/bin/env python3
from pathlib import Path
import re

BRIDGE = Path("/home/robotics-club/agrobot_ws/src/epos2_bridge/epos2_bridge/epos2_j3_bridge.py")
ARM = Path("/home/robotics-club/agrobot_ws/src/epos2_bridge/epos2_bridge/epos2_arm_controller.py")
IFACE_CMAKE = Path("/home/robotics-club/agrobot_ws/src/epos2_bridge_interfaces/CMakeLists.txt")
ARM_CFG = Path("/home/robotics-club/agrobot_ws/src/epos2_bridge/config/epos2_arm_controller.yaml")


def ensure_iface():
    text = IFACE_CMAKE.read_text()
    if '"srv/MoveAbsoluteTimed.srv"' not in text:
        text = text.replace(
            '"srv/MoveAbsolute.srv"\n',
            '"srv/MoveAbsolute.srv"\n  "srv/MoveAbsoluteTimed.srv"\n'
        )
        IFACE_CMAKE.write_text(text)
        print("Patched interface CMakeLists.txt")
    else:
        print("Interface CMakeLists.txt already OK")


def replace_def(text: str, name: str, new_def: str) -> str:
    pat = re.compile(
        rf'(^\s*def {re.escape(name)}\([^\n]*\):[\s\S]*?)(?=^\s*def |\Z)',
        re.MULTILINE
    )
    if pat.search(text):
        return pat.sub(new_def.rstrip() + "\n\n", text, count=1)
    return text.rstrip() + "\n\n" + new_def.rstrip() + "\n"


def patch_bridge():
    text = BRIDGE.read_text()

    # import
    text = re.sub(
        r'from epos2_bridge_interfaces\.srv import ([^\n]+)',
        'from epos2_bridge_interfaces.srv import MoveDelta, MoveAbsolute, MoveAbsoluteTimed',
        text,
        count=1,
    )

    # service creation
    if '"/epos2/j3/move_absolute_timed"' not in text:
        pat = re.compile(
            r'(self\.srv_move_absolute\s*=\s*self\.create_service\(\s*'
            r'MoveAbsolute,\s*'
            r'"/epos2/j3/move_absolute",\s*'
            r'self\._move_absolute_srv,\s*'
            r'\)\s*)',
            re.MULTILINE
        )
        repl = (
            r'\1\n'
            r'        self.srv_move_absolute_timed = self.create_service(\n'
            r'            MoveAbsoluteTimed,\n'
            r'            "/epos2/j3/move_absolute_timed",\n'
            r'            self._move_absolute_timed_srv,\n'
            r'        )\n'
        )
        text, n = pat.subn(repl, text, count=1)
        if n == 0:
            raise RuntimeError("Could not locate move_absolute service creation block in bridge")

    timed_def = """
    def send_test_move_absolute_timed(self, target_rad: float, duration_sec: float) -> bool:
        if not self.ipm_armed:
            self.get_logger().warning("IPM not armed; call arm_ipm_hold() first")
            return False

        current_pos = self.sdo_read(IDX_POSITION_ACTUAL, 0, warn=False)
        if current_pos is None:
            self.get_logger().error("Failed reading current position for timed absolute move")
            return False

        current_pos_qc = to_signed_32(current_pos)
        current_rad = self.kin.motor_qc_to_joint_rad(current_pos_qc)
        target_qc = self.kin.joint_rad_to_motor_qc(float(target_rad))
        delta_qc = target_qc - current_pos_qc

        duration_sec = max(0.15, float(duration_sec))
        seg_ms = max(50, int(round(duration_sec * 1000.0)))
        seg_s = seg_ms / 1000.0

        motor_rpm = max(
            1,
            abs(
                int(
                    round(
                        delta_qc * 60.0
                        / (self.kin.encoder_qc_per_motor_rev * seg_s)
                    )
                )
            ),
        )
        if delta_qc < 0:
            motor_rpm = -motor_rpm

        p_start = PVTPoint(time_ms=seg_ms, velocity_rpm=0, position_qc=current_pos_qc)
        p_move  = PVTPoint(time_ms=seg_ms, velocity_rpm=motor_rpm, position_qc=target_qc)
        p_hold  = PVTPoint(time_ms=seg_ms, velocity_rpm=0, position_qc=target_qc)

        self._set_bridge_state(BridgeState.MOVING, "queueing timed absolute move")

        try:
            self.get_logger().info(
                f"Timed absolute move target_rad={target_rad:.6f} "
                f"current_rad={current_rad:.6f} duration_sec={duration_sec:.3f} "
                f"current_qc={current_pos_qc} target_qc={target_qc} motor_rpm={motor_rpm}"
            )

            self.send_rpdo1_interpolation_record(p_start, verbose=True)
            time.sleep(0.002)
            self.send_rpdo1_interpolation_record(p_move, verbose=True)
            time.sleep(0.002)

            for _ in range(6):
                self.send_rpdo1_interpolation_record(p_hold, verbose=False)
                time.sleep(0.002)

            self.last_hold_qc = target_qc

            time.sleep(duration_sec + 0.25)

            final_pos = self.sdo_read(IDX_POSITION_ACTUAL, 0, warn=False)
            if final_pos is None:
                self.get_logger().error("Failed reading final position after timed move")
                self._set_bridge_state(BridgeState.FAULTED, "no final position after timed move")
                self.ipm_armed = False
                return False

            final_qc = to_signed_32(final_pos)
            final_rad = self.kin.motor_qc_to_joint_rad(final_qc)
            err_rad = float(target_rad) - float(final_rad)

            self.get_logger().info(
                f"Timed absolute result final_rad={final_rad:.6f} "
                f"target_rad={target_rad:.6f} err_rad={err_rad:.6f}"
            )

            sw = self.sdo_read(IDX_STATUSWORD, 0, warn=False)
            if sw is not None and ((sw >> SW_FAULT_BIT) & 0x1):
                self._set_bridge_state(BridgeState.FAULTED, "fault after timed move")
                self.ipm_armed = False
                return False

            if abs(err_rad) > 0.03:
                self.get_logger().error(
                    f"Timed absolute move out of tolerance: err_rad={err_rad:.6f}"
                )
                self._set_bridge_state(BridgeState.IPM_ARMED, "timed move ended out of tolerance")
                return False

            self._set_bridge_state(BridgeState.IPM_ARMED, "returned to hold after timed move")
            return True

        except Exception as exc:
            self.get_logger().error(f"Failed timed absolute move: {exc}")
            self._set_bridge_state(BridgeState.FAULTED, "exception during timed move")
            self.ipm_armed = False
            return False
    """
    text = replace_def(text, "send_test_move_absolute_timed", timed_def)

    timed_srv = """
    def _move_absolute_timed_srv(self, request, response):
        ok = self.send_test_move_absolute_timed(float(request.target_rad), float(request.duration_sec))
        response.success = ok
        response.message = (
            f"move_absolute_timed succeeded for target_rad={request.target_rad}, duration_sec={request.duration_sec}"
            if ok else
            f"move_absolute_timed failed for target_rad={request.target_rad}, duration_sec={request.duration_sec}"
        )
        return response
    """
    text = replace_def(text, "_move_absolute_timed_srv", timed_srv)

    BRIDGE.write_text(text)
    print("Patched bridge")


def patch_arm():
    text = ARM.read_text()

    # import
    text = re.sub(
        r'from epos2_bridge_interfaces\.srv import ([^\n]+)',
        'from epos2_bridge_interfaces.srv import MoveDelta, MoveAbsolute, MoveAbsoluteTimed',
        text,
        count=1,
    )

    # move_clients
    text = re.sub(
        r'self\.create_client\(\s*MoveAbsolute\s*,\s*s,\s*callback_group=self\.cb_group\s*\)',
        'self.create_client(MoveAbsoluteTimed, s, callback_group=self.cb_group)',
        text
    )
    text = re.sub(
        r'self\.create_client\(\s*MoveDelta\s*,\s*s,\s*callback_group=self\.cb_group\s*\)',
        'self.create_client(MoveAbsoluteTimed, s, callback_group=self.cb_group)',
        text
    )

    move_helper = """
    def _call_move_absolute_timed(self, client, target_rad: float, duration_sec: float, label: str) -> bool:
        req = MoveAbsoluteTimed.Request()
        req.target_rad = float(target_rad)
        req.duration_sec = float(duration_sec)
        future = client.call_async(req)
        result = self._wait_future(future, timeout_sec=max(60.0, duration_sec + 5.0))
        if result is None:
            self.get_logger().error(f"{label}: timeout")
            return False
        if not result.success:
            self.get_logger().error(f"{label}: failed: {result.message}")
            return False
        self.get_logger().info(f"{label}: {result.message}")
        return True
    """
    text = replace_def(text, "_call_move_absolute_timed", move_helper)

    execute_goal = """
    async def _execute_goal(self, goal_handle):
        result = FollowJointTrajectory.Result()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = self.joint_names

        if not self._wait_for_current_state():
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "No current joint state available"
            goal_handle.abort()
            return result

        with self.joint_pos_lock:
            commanded = {j: self.latest_joint_pos[j] for j in self.joint_names}

        for j in self.joint_names:
            if not self._call_trigger(self.arm_clients[j], f"{j} arm_ipm"):
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = f"Failed to arm {j}"
                goal_handle.abort()
                return result

        points = list(goal_handle.request.trajectory.points)
        self.get_logger().info(f"Received {len(points)} trajectory points from MoveIt")

        reduced = []
        last_t = -1.0
        for pt in points:
            t = float(pt.time_from_start.sec) + float(pt.time_from_start.nanosec) * 1e-9
            if not reduced or (t - last_t) >= 0.2:
                reduced.append(pt)
                last_t = t
        if not reduced or reduced[-1] is not points[-1]:
            reduced.append(points[-1])

        self.get_logger().info(f"Service-backed timed mode: executing {len(reduced)} reduced points")
        points = reduced

        start_time = time.monotonic()

        for p_idx, pt in enumerate(points):
            if goal_handle.is_cancel_requested:
                for j in self.joint_names:
                    self._call_trigger(self.disarm_clients[j], f"{j} disarm_ipm after cancel")
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "Goal canceled"
                return result

            if len(pt.positions) != len(self.joint_names):
                for j in self.joint_names:
                    self._call_trigger(self.disarm_clients[j], f"{j} disarm_ipm after invalid point")
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = f"Point {p_idx} has wrong positions length"
                goal_handle.abort()
                return result

            target_map = {j: float(pos) for j, pos in zip(self.joint_names, pt.positions)}
            self.get_logger().info(f"Executing absolute target_map={target_map}")

            pt_time = float(pt.time_from_start.sec) + float(pt.time_from_start.nanosec) * 1e-9
            if p_idx == 0:
                prev_time = 0.0
            else:
                prev_pt = points[p_idx - 1]
                prev_time = float(prev_pt.time_from_start.sec) + float(prev_pt.time_from_start.nanosec) * 1e-9
            dt = max(0.15, pt_time - prev_time)

            for j in self.joint_names:
                ok = self._call_move_absolute_timed(
                    self.move_clients[j],
                    target_map[j],
                    dt,
                    f"{j} move_absolute_timed point {p_idx}",
                )
                if not ok:
                    for jj in self.joint_names:
                        self._call_trigger(self.disarm_clients[jj], f"{jj} disarm_ipm after move failure")
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = f"Timed absolute move failed for {j} at point {p_idx}"
                    goal_handle.abort()
                    return result

            reached = self._wait_until_targets_reached(
                target_map=target_map,
                timeout_sec=max(dt + 1.0, 2.0),
                tolerance_rad=self.goal_position_tolerance_rad,
            )

            with self.joint_pos_lock:
                actual_positions = [self.latest_joint_pos.get(j, 0.0) for j in self.joint_names]
                actual_velocities = [self.latest_joint_vel.get(j, 0.0) for j in self.joint_names]

            feedback.desired = pt
            feedback.actual = pt.__class__(
                positions=actual_positions,
                velocities=actual_velocities,
            )
            feedback.error = pt.__class__(
                positions=[target_map[j] - self.latest_joint_pos.get(j, 0.0) for j in self.joint_names],
                velocities=[0.0 for _ in self.joint_names],
            )
            goal_handle.publish_feedback(feedback)

            if not reached:
                for j in self.joint_names:
                    self._call_trigger(self.disarm_clients[j], f"{j} disarm_ipm after point timeout")
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = f"Point {p_idx} not reached within tolerance/time budget"
                goal_handle.abort()
                return result

            commanded = target_map

        with self.joint_pos_lock:
            final_actual = {j: self.latest_joint_pos.get(j, 0.0) for j in self.joint_names}

        bad = []
        for j in self.joint_names:
            if abs(final_actual[j] - commanded[j]) > self.goal_position_tolerance_rad:
                bad.append((j, final_actual[j], commanded[j]))

        for j in self.joint_names:
            self._call_trigger(self.disarm_clients[j], f"{j} disarm_ipm after goal")

        if bad:
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = "Final position tolerance violated: " + ", ".join(
                [f"{j}: actual={a:.6f}, target={t:.6f}" for j, a, t in bad]
            )
            goal_handle.abort()
            return result

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Success"
        return result
    """
    text = replace_def(text, "_execute_goal", execute_goal)

    ARM.write_text(text)
    print("Patched arm controller")


def patch_cfg():
    ARM_CFG.write_text("""epos2_arm_controller:
  ros__parameters:
    joint_names: ["joint3"]
    arm_services: ["/epos2/j3/arm_ipm"]
    disarm_services: ["/epos2/j3/disarm_ipm"]
    move_services: ["/epos2/j3/move_absolute_timed"]
    goal_position_tolerance_rad: 0.03
    wait_for_joint_state_sec: 5.0
""")
    print("Patched arm controller config")


ensure_iface()
patch_bridge()
patch_arm()
patch_cfg()
print("All timed-mode patches applied")

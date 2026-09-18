from pathlib import Path

p = Path('/home/robotics-club/agrobot_ws/src/epos2_bridge/epos2_bridge/epos2_j3_bridge.py')
text = p.read_text()

# 1) Add stream state members
old = '        self.bridge_state = BridgeState.IDLE\n'
new = '''        self.bridge_state = BridgeState.IDLE
        self.stream_lock = threading.Lock()
'''
if old not in text:
    raise RuntimeError("Could not find bridge_state init marker")
text = text.replace(old, new, 1)

# 2) Add reduced trajectory subscription before action server section
marker = '        # ---------------- Action server ----------------\n'
insert = '''
        self.sub_reduced_traj = self.create_subscription(
            Float64MultiArray,
            "/epos2/j3/reduced_traj",
            self._reduced_traj_cb,
            qos,
        )

'''
if marker not in text:
    raise RuntimeError("Could not find action server marker")
text = text.replace(marker, insert + marker, 1)

# 3) Restrict keepalive to only when idle-and-armed
old = '''        if self.bridge_state not in (BridgeState.IPM_ARMED, BridgeState.MOVING):
            return
'''
new = '''        if self.bridge_state != BridgeState.IPM_ARMED:
            return
'''
if old not in text:
    raise RuntimeError("Could not find keepalive bridge_state check")
text = text.replace(old, new, 1)

# 4) Add reduced trajectory methods before PDO runtime section
marker = '    # ---------------- PDO runtime ----------------\n'
methods = r'''
    def _reduced_traj_cb(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) == 0 or (len(data) % 2) != 0:
            self.get_logger().warning(
                "Ignoring /reduced_traj: expected [target0, dt0, target1, dt1, ...]"
            )
            return

        targets = [float(data[i]) for i in range(0, len(data), 2)]
        durs = [float(data[i]) for i in range(1, len(data), 2)]

        ok = self.execute_reduced_trajectory(targets, durs)
        self.get_logger().info(f"/reduced_traj result={ok}")

    def _segment_points_between(
        self,
        start_qc: int,
        target_qc: int,
        duration_sec: float,
    ) -> List[PVTPoint]:
        duration_sec = max(0.02, float(duration_sec))
        total_ms = int(round(duration_sec * 1000.0))

        min_seg_ms = 20
        max_seg_ms = 80

        n_segments = max(1, math.ceil(total_ms / max_seg_ms))
        seg_ms = max(min_seg_ms, min(max_seg_ms, int(round(total_ms / n_segments))))
        seg_s = seg_ms / 1000.0

        pts: List[PVTPoint] = []
        prev_qc = start_qc

        for k in range(1, n_segments + 1):
            frac = k / n_segments
            seg_target_qc = int(round(start_qc + frac * (target_qc - start_qc)))
            seg_delta_qc = seg_target_qc - prev_qc

            if seg_delta_qc == 0:
                motor_rpm = 0
            else:
                motor_rpm = int(
                    math.ceil(
                        abs(seg_delta_qc) * 60.0
                        / (self.kin.encoder_qc_per_motor_rev * seg_s)
                    )
                )
                motor_rpm = max(1, motor_rpm)
                if seg_delta_qc < 0:
                    motor_rpm = -motor_rpm

            pts.append(
                PVTPoint(
                    time_ms=seg_ms,
                    velocity_rpm=motor_rpm,
                    position_qc=seg_target_qc,
                )
            )
            prev_qc = seg_target_qc

        return pts

    def execute_reduced_trajectory(self, target_rads: List[float], duration_secs: List[float]) -> bool:
        if not self.ipm_armed:
            self.get_logger().warning("IPM not armed; call arm_ipm first")
            return False

        if len(target_rads) != len(duration_secs) or len(target_rads) == 0:
            self.get_logger().error("execute_reduced_trajectory got mismatched or empty arrays")
            return False

        with self.stream_lock:
            if self.bridge_state not in (BridgeState.IPM_ARMED, BridgeState.MOVING):
                self.get_logger().warning(f"Refusing reduced trajectory in state {self.bridge_state.name}")
                return False

            current_pos = self.sdo_read(IDX_POSITION_ACTUAL, 0, warn=False)
            if current_pos is None:
                self.get_logger().error("Failed reading current position for reduced trajectory")
                return False

            current_qc = to_signed_32(current_pos)
            prev_qc = current_qc

            all_points: List[PVTPoint] = []
            for target_rad, dt in zip(target_rads, duration_secs):
                target_qc = self.kin.joint_rad_to_motor_qc(float(target_rad))
                all_points.extend(self._segment_points_between(prev_qc, target_qc, float(dt)))
                prev_qc = target_qc

            if not all_points:
                self.get_logger().warning("Reduced trajectory produced no executable points")
                return False

            self._set_bridge_state(BridgeState.MOVING, "executing reduced trajectory")

            try:
                # Small lead-in hold
                lead = PVTPoint(time_ms=40, velocity_rpm=0, position_qc=current_qc)
                self.send_rpdo1_interpolation_record(lead, verbose=True)
                time.sleep(0.002)

                for i, pt in enumerate(all_points):
                    self.send_rpdo1_interpolation_record(pt, verbose=(i < 6))
                    time.sleep(0.002)

                final_qc = all_points[-1].position_qc
                self.last_hold_qc = final_qc

                hold = PVTPoint(time_ms=40, velocity_rpm=0, position_qc=final_qc)
                for _ in range(8):
                    self.send_rpdo1_interpolation_record(hold, verbose=False)
                    time.sleep(0.002)

                total_wait = sum(max(0.02, float(d)) for d in duration_secs) + 1.0
                deadline = time.monotonic() + total_wait
                final_target_rad = target_rads[-1]

                while time.monotonic() < deadline:
                    sw = self.sdo_read(IDX_STATUSWORD, 0, warn=False)
                    if sw is not None and ((sw >> SW_FAULT_BIT) & 0x1):
                        self.get_logger().error(f"Fault during reduced trajectory, sw=0x{sw:04X}")
                        self.ipm_armed = False
                        self._set_bridge_state(BridgeState.FAULTED, "fault during reduced trajectory")
                        return False

                    pos = self.sdo_read(IDX_POSITION_ACTUAL, 0, warn=False)
                    if pos is None:
                        time.sleep(0.02)
                        continue

                    pos_qc = to_signed_32(pos)
                    pos_rad = self.kin.motor_qc_to_joint_rad(pos_qc)
                    err_rad = float(final_target_rad) - float(pos_rad)

                    if abs(err_rad) <= 0.03:
                        self._set_bridge_state(BridgeState.IPM_ARMED, "reduced trajectory complete")
                        return True

                    time.sleep(0.02)

                self.get_logger().error("Reduced trajectory timed out before reaching final tolerance")
                self._set_bridge_state(BridgeState.IPM_ARMED, "reduced trajectory timeout")
                return False

            except Exception as exc:
                self.get_logger().error(f"Reduced trajectory exception: {exc}")
                self.ipm_armed = False
                self._set_bridge_state(BridgeState.FAULTED, "reduced trajectory exception")
                return False

'''
if marker not in text:
    raise RuntimeError("Could not find PDO runtime marker")
text = text.replace(marker, methods + marker, 1)

p.write_text(text)
print(f"patched {p}")

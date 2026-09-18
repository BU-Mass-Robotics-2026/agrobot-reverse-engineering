#!/usr/bin/env python3
from pathlib import Path
import re

p = Path("/home/robotics-club/agrobot_ws/src/epos2_bridge/epos2_bridge/epos2_j3_bridge.py")
text = p.read_text()

new_def = """
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
        delta_qc_total = target_qc - current_pos_qc

        duration_sec = max(0.15, float(duration_sec))

        # Current packet builder clearly rejects 1000 ms. Keep segments <= 250 ms.
        max_seg_ms = 250
        min_seg_ms = 50
        lead_ms = 80

        total_ms = int(round(duration_sec * 1000.0))
        n_segments = max(1, math.ceil(total_ms / max_seg_ms))
        seg_duration_sec = duration_sec / n_segments
        seg_ms = max(min_seg_ms, min(max_seg_ms, int(round(seg_duration_sec * 1000.0))))
        seg_s = seg_ms / 1000.0

        self._set_bridge_state(BridgeState.MOVING, "queueing timed absolute move")

        try:
            self.get_logger().info(
                f"Timed absolute move target_rad={target_rad:.6f} "
                f"current_rad={current_rad:.6f} duration_sec={duration_sec:.3f} "
                f"lead_ms={lead_ms} seg_ms={seg_ms} n_segments={n_segments} "
                f"current_qc={current_pos_qc} target_qc={target_qc}"
            )

            # Short lead-in hold so motion starts almost immediately
            p_start = PVTPoint(time_ms=lead_ms, velocity_rpm=0, position_qc=current_pos_qc)
            self.send_rpdo1_interpolation_record(p_start, verbose=True)
            time.sleep(0.002)

            prev_qc = current_pos_qc

            # Break the move into several absolute subtargets
            for k in range(1, n_segments + 1):
                frac = k / n_segments
                seg_target_qc = int(round(current_pos_qc + delta_qc_total * frac))
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

                p_move = PVTPoint(
                    time_ms=seg_ms,
                    velocity_rpm=motor_rpm,
                    position_qc=seg_target_qc,
                )
                self.send_rpdo1_interpolation_record(p_move, verbose=True)
                time.sleep(0.002)
                prev_qc = seg_target_qc

            # Final hold points
            p_hold = PVTPoint(time_ms=lead_ms, velocity_rpm=0, position_qc=target_qc)
            for _ in range(6):
                self.send_rpdo1_interpolation_record(p_hold, verbose=False)
                time.sleep(0.002)

            self.last_hold_qc = target_qc

            # Wait for the full commanded duration plus a little settle time
            time.sleep(duration_sec + 0.35)

            deadline = time.monotonic() + 1.0
            final_rad = None
            err_rad = None

            while time.monotonic() < deadline:
                final_pos = self.sdo_read(IDX_POSITION_ACTUAL, 0, warn=False)
                if final_pos is None:
                    time.sleep(0.05)
                    continue

                final_qc = to_signed_32(final_pos)
                final_rad = self.kin.motor_qc_to_joint_rad(final_qc)
                err_rad = float(target_rad) - float(final_rad)

                if abs(err_rad) <= 0.03:
                    break

                time.sleep(0.05)

            if final_rad is None or err_rad is None:
                self.get_logger().error("Failed reading final position after timed move")
                self._set_bridge_state(BridgeState.FAULTED, "no final position after timed move")
                self.ipm_armed = False
                return False

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

pat = re.compile(
    r'(^\s*def send_test_move_absolute_timed\(self, target_rad: float, duration_sec: float\) -> bool:[\s\S]*?)(?=^\s*def |\Z)',
    re.MULTILINE
)

new_text, n = pat.subn(new_def.rstrip() + "\\n\\n", text, count=1)
if n == 0:
    raise RuntimeError("Could not find send_test_move_absolute_timed() in epos2_j3_bridge.py")

p.write_text(new_text)
print("Patched send_test_move_absolute_timed()")

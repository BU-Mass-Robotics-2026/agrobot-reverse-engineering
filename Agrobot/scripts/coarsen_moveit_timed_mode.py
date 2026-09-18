#!/usr/bin/env python3
from pathlib import Path
import re

p = Path("/home/robotics-club/agrobot_ws/src/epos2_bridge/epos2_bridge/epos2_arm_controller.py")
text = p.read_text()

new_def = """
    async def _execute_goal(self, goal_handle):
        result = FollowJointTrajectory.Result()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = self.joint_names

        if not self._wait_for_current_state():
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "No current joint state available"
            goal_handle.abort()
            return result

        for j in self.joint_names:
            if not self._call_trigger(self.arm_clients[j], f"{j} arm_ipm"):
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = f"Failed to arm {j}"
                goal_handle.abort()
                return result

        points = list(goal_handle.request.trajectory.points)
        self.get_logger().info(f"Received {len(points)} trajectory points from MoveIt")

        # Coarsen aggressively for visible motion tonight:
        # keep points that are >=0.5s apart OR >=0.02 rad apart, always keep last point.
        reduced = []
        last_t = None
        last_pos = None

        for pt in points:
            t = float(pt.time_from_start.sec) + float(pt.time_from_start.nanosec) * 1e-9
            pos = float(pt.positions[0])

            keep = False
            if not reduced:
                keep = True
            else:
                dt = t - last_t
                dpos = abs(pos - last_pos)
                if dt >= 0.5 or dpos >= 0.02:
                    keep = True

            if keep:
                reduced.append(pt)
                last_t = t
                last_pos = pos

        if not reduced or reduced[-1] is not points[-1]:
            reduced.append(points[-1])

        self.get_logger().info(f"Service-backed timed mode: executing {len(reduced)} reduced points")
        points = reduced

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
            dt = max(0.20, pt_time - prev_time)

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

        with self.joint_pos_lock:
            final_actual = {j: self.latest_joint_pos.get(j, 0.0) for j in self.joint_names}

        bad = []
        for j in self.joint_names:
            if abs(final_actual[j] - target_map[j]) > self.goal_position_tolerance_rad:
                bad.append((j, final_actual[j], target_map[j]))

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

pat = re.compile(
    r'(^\s*async def _execute_goal\(self, goal_handle\):[\s\S]*?)(?=^\s*def |\Z)',
    re.MULTILINE
)

new_text, n = pat.subn(new_def.rstrip() + "\\n\\n", text, count=1)
if n == 0:
    raise RuntimeError("Could not find _execute_goal() in epos2_arm_controller.py")

p.write_text(new_text)
print("Patched _execute_goal() with coarser reduction")

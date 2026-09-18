#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class Epos2FjtFanout(Node):
    """Fan out one MoveIt FollowJointTrajectory goal to per-joint FJT action servers.

    This replaces the older service-backed epos2_arm_controller path. It does not call
    arm_ipm / move_absolute_timed / disarm_ipm. Each child EPOS2 bridge receives a normal
    one-joint FollowJointTrajectory goal and handles its own IPM lifecycle:
    prepare -> prefill -> 0x001F -> stream -> dt=0 -> Position Mode hold.
    """

    def __init__(self) -> None:
        super().__init__("epos2_fjt_fanout")

        self.declare_parameter("controller_action_name", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("controlled_joint_names", ["joint2", "joint3", "joint4", "joint5"])
        self.declare_parameter(
            "child_action_names",
            [
                "/j2_position_controller/follow_joint_trajectory",
                "/j3_position_controller/follow_joint_trajectory",
                "/j4_position_controller/follow_joint_trajectory",
                "/j5_position_controller/follow_joint_trajectory",
            ],
        )
        self.declare_parameter("allow_extra_goal_joints", True)
        self.declare_parameter("require_all_controlled_joints", True)
        self.declare_parameter("child_server_timeout_sec", 10.0)
        self.declare_parameter("result_timeout_margin_sec", 5.0)
        self.declare_parameter("child_sync_delay_sec", 0.25)

        self.controller_action_name = str(self.get_parameter("controller_action_name").value)
        self.controlled_joint_names: List[str] = list(self.get_parameter("controlled_joint_names").value)
        self.child_action_names: List[str] = list(self.get_parameter("child_action_names").value)
        self.allow_extra_goal_joints = bool(self.get_parameter("allow_extra_goal_joints").value)
        self.require_all_controlled_joints = bool(self.get_parameter("require_all_controlled_joints").value)
        self.child_server_timeout_sec = float(self.get_parameter("child_server_timeout_sec").value)
        self.result_timeout_margin_sec = float(self.get_parameter("result_timeout_margin_sec").value)
        self.child_sync_delay_sec = float(self.get_parameter("child_sync_delay_sec").value)

        if len(self.controlled_joint_names) != len(self.child_action_names):
            raise RuntimeError(
                "controlled_joint_names and child_action_names must have equal length"
            )

        self.cb_group = ReentrantCallbackGroup()

        self.child_clients: Dict[str, ActionClient] = {
            joint: ActionClient(
                self,
                FollowJointTrajectory,
                action_name,
                callback_group=self.cb_group,
            )
            for joint, action_name in zip(self.controlled_joint_names, self.child_action_names)
        }

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.controller_action_name,
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            "EPOS2 FJT fanout online: "
            f"controller_action={self.controller_action_name} "
            f"controlled_joints={self.controlled_joint_names} "
            f"child_actions={self.child_action_names} "
            f"allow_extra_goal_joints={self.allow_extra_goal_joints} "
            f"child_sync_delay_sec={self.child_sync_delay_sec}"
        )

    async def _await_future(self, future, timeout_sec: Optional[float], label: str):
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
        while rclpy.ok():
            if future.done():
                return future.result()
            if deadline is not None and time.monotonic() > deadline:
                self.get_logger().error(f"{label}: timeout after {timeout_sec:.2f}s")
                return None
            await asyncio.sleep(0.01)
        return None

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> int:
        incoming = list(goal_request.trajectory.joint_names)

        if not incoming:
            self.get_logger().warning("Rejecting fanout goal: empty joint_names")
            return GoalResponse.REJECT

        missing = [j for j in self.controlled_joint_names if j not in incoming]
        extras = [j for j in incoming if j not in self.controlled_joint_names]

        if missing and self.require_all_controlled_joints:
            self.get_logger().warning(
                f"Rejecting fanout goal: missing controlled joints {missing}; incoming={incoming}"
            )
            return GoalResponse.REJECT

        if extras and not self.allow_extra_goal_joints:
            self.get_logger().warning(
                f"Rejecting fanout goal: extra unmanaged joints {extras}; incoming={incoming}"
            )
            return GoalResponse.REJECT

        if len(goal_request.trajectory.points) < 1:
            self.get_logger().warning("Rejecting fanout goal: no trajectory points")
            return GoalResponse.REJECT

        if extras:
            self.get_logger().warning(
                f"Fanout accepting goal with unmanaged extra joints {extras}; "
                "they will be ignored by this EPOS2-only fanout instance"
            )

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> int:
        return CancelResponse.ACCEPT

    def _parent_goal_with_sync_stamp(self, parent_goal: FollowJointTrajectory.Goal) -> FollowJointTrajectory.Goal:
        """Ensure child goals share a common future start time when parent stamp is zero."""
        stamp = parent_goal.trajectory.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            return parent_goal

        if self.child_sync_delay_sec <= 0.0:
            return parent_goal

        now_ns = self.get_clock().now().nanoseconds
        target_ns = now_ns + int(self.child_sync_delay_sec * 1e9)

        parent_goal.trajectory.header.stamp.sec = int(target_ns // 1_000_000_000)
        parent_goal.trajectory.header.stamp.nanosec = int(target_ns % 1_000_000_000)

        self.get_logger().info(
            f"Fanout assigned sync header.stamp +{self.child_sync_delay_sec*1000.0:.1f}ms "
            "for child synchronization"
        )
        return parent_goal

    def _make_child_goal(
        self,
        parent_goal: FollowJointTrajectory.Goal,
        joint_name: str,
        joint_index: int,
    ) -> FollowJointTrajectory.Goal:
        child = FollowJointTrajectory.Goal()
        child.trajectory.header = parent_goal.trajectory.header
        child.trajectory.joint_names = [joint_name]

        for pt in parent_goal.trajectory.points:
            cpt = JointTrajectoryPoint()

            if joint_index < len(pt.positions):
                cpt.positions = [float(pt.positions[joint_index])]
            else:
                cpt.positions = []

            if joint_index < len(pt.velocities):
                cpt.velocities = [float(pt.velocities[joint_index])]
            else:
                cpt.velocities = []

            if joint_index < len(pt.accelerations):
                cpt.accelerations = [float(pt.accelerations[joint_index])]
            else:
                cpt.accelerations = []

            if joint_index < len(pt.effort):
                cpt.effort = [float(pt.effort[joint_index])]
            else:
                cpt.effort = []

            cpt.time_from_start = pt.time_from_start
            child.trajectory.points.append(cpt)

        child.goal_time_tolerance = parent_goal.goal_time_tolerance

        # Path/goal tolerances are per-joint arrays. Keep matching entries only.
        for tol in parent_goal.path_tolerance:
            if tol.name == joint_name:
                child.path_tolerance.append(tol)
        for tol in parent_goal.goal_tolerance:
            if tol.name == joint_name:
                child.goal_tolerance.append(tol)

        return child

    def _trajectory_duration_sec(self, goal: FollowJointTrajectory.Goal) -> float:
        if not goal.trajectory.points:
            return 0.0
        last = goal.trajectory.points[-1].time_from_start
        return float(last.sec) + float(last.nanosec) * 1e-9

    async def _send_child_goal(
        self,
        parent_goal_handle,
        parent_goal: FollowJointTrajectory.Goal,
        joint_name: str,
        child_action_name: str,
        joint_index: int,
    ) -> Tuple[str, bool, str]:
        client = self.child_clients[joint_name]

        if not client.wait_for_server(timeout_sec=self.child_server_timeout_sec):
            return joint_name, False, f"{joint_name}: child action server unavailable: {child_action_name}"

        child_goal = self._make_child_goal(parent_goal, joint_name, joint_index)
        self.get_logger().info(
            f"Fanout sending {joint_name} -> {child_action_name} "
            f"points={len(child_goal.trajectory.points)}"
        )

        send_future = client.send_goal_async(child_goal)
        child_goal_handle = await self._await_future(
            send_future,
            timeout_sec=self.child_server_timeout_sec,
            label=f"{joint_name} send_goal",
        )

        if child_goal_handle is None:
            return joint_name, False, f"{joint_name}: send_goal timeout"

        if not child_goal_handle.accepted:
            return joint_name, False, f"{joint_name}: child goal rejected"

        result_future = child_goal_handle.get_result_async()
        result_timeout = max(10.0, self._trajectory_duration_sec(parent_goal) + self.result_timeout_margin_sec)

        while rclpy.ok():
            if parent_goal_handle.is_cancel_requested:
                self.get_logger().warning(f"Parent goal canceled; canceling child {joint_name}")
                cancel_future = child_goal_handle.cancel_goal_async()
                await self._await_future(cancel_future, timeout_sec=2.0, label=f"{joint_name} cancel")
                return joint_name, False, f"{joint_name}: canceled by parent"

            if result_future.done():
                wrapped = result_future.result()
                if wrapped is None:
                    return joint_name, False, f"{joint_name}: empty child result"

                child_result = wrapped.result
                if child_result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                    return (
                        joint_name,
                        False,
                        f"{joint_name}: child failed code={child_result.error_code} "
                        f"msg={child_result.error_string}",
                    )

                return joint_name, True, f"{joint_name}: success"

            if result_timeout is not None:
                # Convert to absolute deadline lazily by storing on future object.
                if not hasattr(result_future, "_fanout_deadline"):
                    result_future._fanout_deadline = time.monotonic() + result_timeout
                if time.monotonic() > result_future._fanout_deadline:
                    return joint_name, False, f"{joint_name}: child result timeout"

            await asyncio.sleep(0.01)

        return joint_name, False, f"{joint_name}: rclpy shutdown"

    async def _execute_goal(self, goal_handle):
        parent_goal = goal_handle.request
        parent_goal = self._parent_goal_with_sync_stamp(parent_goal)
        incoming = list(parent_goal.trajectory.joint_names)

        result = FollowJointTrajectory.Result()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = incoming

        missing = [j for j in self.controlled_joint_names if j not in incoming]
        if missing and self.require_all_controlled_joints:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"Missing controlled joints: {missing}"
            goal_handle.abort()
            return result

        # Build child tasks for controlled joints that are present in the incoming trajectory.
        tasks = []
        selected = []
        for joint, action_name in zip(self.controlled_joint_names, self.child_action_names):
            if joint not in incoming:
                self.get_logger().warning(f"Skipping controlled joint absent from goal: {joint}")
                continue
            idx = incoming.index(joint)
            selected.append(joint)
            tasks.append(
                asyncio.create_task(
                    self._send_child_goal(goal_handle, parent_goal, joint, action_name, idx)
                )
            )

        if not tasks:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "No configured EPOS2 joints were present in the incoming trajectory"
            goal_handle.abort()
            return result

        self.get_logger().info(
            f"Fanout executing parent goal: incoming_joints={incoming} selected_epos2_joints={selected}"
        )

        child_results = await asyncio.gather(*tasks, return_exceptions=True)

        failures = []
        for item in child_results:
            if isinstance(item, Exception):
                failures.append(f"exception: {item}")
                continue
            joint, ok, msg = item
            if ok:
                self.get_logger().info(msg)
            else:
                self.get_logger().error(msg)
                failures.append(msg)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "Fanout goal canceled; child cancellations requested"
            return result

        if failures:
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = "; ".join(failures)
            goal_handle.abort()
            return result

        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Fanout succeeded"
        goal_handle.succeed()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Epos2FjtFanout()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

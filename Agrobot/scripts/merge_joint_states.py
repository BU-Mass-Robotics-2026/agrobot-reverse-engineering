#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        self.pub = self.create_publisher(JointState, '/joint_states', 20)
        self.sub = self.create_subscription(
            JointState,
            '/epos2/j3/joint_states_raw',
            self.raw_cb,
            20,
        )

        self.latest_joint3_pos = 0.0
        self.latest_joint3_vel = 0.0
        self.have_joint3 = False

        self.timer = self.create_timer(0.05, self.tick)

    def raw_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == 'joint3':
                if i < len(msg.position):
                    self.latest_joint3_pos = float(msg.position[i])
                if i < len(msg.velocity):
                    self.latest_joint3_vel = float(msg.velocity[i])
                else:
                    self.latest_joint3_vel = 0.0
                self.have_joint3 = True

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = [0.0, 0.0, self.latest_joint3_pos, 0.0, 0.0, 0.0]
        msg.velocity = [0.0, 0.0, self.latest_joint3_vel, 0.0, 0.0, 0.0]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

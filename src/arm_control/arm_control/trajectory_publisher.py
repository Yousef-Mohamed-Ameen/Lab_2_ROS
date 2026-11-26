#!/usr/bin/env python3
"""Publishes a looping demo JointTrajectory to /arm_controller/joint_trajectory."""
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
# radians, within the +/-pi/2 joint limits
WAYPOINTS = [
    [0.0, 0.0, 0.0, 0.0],
    [0.8, 0.5, 0.6, 0.3],
    [-0.8, -0.5, 0.9, -0.3],
    [0.0, 0.6, -0.7, 0.8],
    [0.0, 0.0, 0.0, 0.0],
]


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.declare_parameter('segment_time', 3.0)   # seconds between waypoints
        self.declare_parameter('loop', True)
        self.seg = float(self.get_parameter('segment_time').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.total = self.seg * len(WAYPOINTS)
        self.timer = self.create_timer(self.total + 1.0, self.publish)
        self.first = self.create_timer(1.0, self.publish_first)

    def publish_first(self):
        self.first.cancel()
        self.publish()

    def publish(self):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        for i, wp in enumerate(WAYPOINTS, start=1):
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in wp]
            t = self.seg * i
            p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            msg.points.append(p)
        self.pub.publish(msg)
        self.get_logger().info(f'Published trajectory with {len(msg.points)} points')
        if not self.loop:
            self.timer.cancel()


def main():
    rclpy.init()
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

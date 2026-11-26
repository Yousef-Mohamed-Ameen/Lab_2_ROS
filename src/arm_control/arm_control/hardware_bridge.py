#!/usr/bin/env python3
"""ROS 2 -> microcontroller bridge (USB serial).

Subscribes to /arm_controller/joint_trajectory, linearly interpolates between
trajectory points at `rate_hz`, converts radians to servo degrees
(deg = centre + direction * degrees(rad), clamped to [min, max]) and sends

        J,<a1>,<a2>,<a3>,<a4>\n        (angles in degrees, 1 decimal)

to the firmware.  Joints absent from a message keep their previous value.
"""
import math
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4'])
        self.declare_parameter('servo_center_deg', [90.0, 90.0, 90.0, 90.0])
        self.declare_parameter('servo_direction', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('servo_min_deg', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('servo_max_deg', [180.0, 180.0, 180.0, 180.0])
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('dry_run', False)

        gp = lambda n: self.get_parameter(n).value
        self.joints = list(gp('joint_names'))
        self.center = list(gp('servo_center_deg'))
        self.direction = list(gp('servo_direction'))
        self.lo = list(gp('servo_min_deg'))
        self.hi = list(gp('servo_max_deg'))
        n = len(self.joints)
        if not all(len(x) == n for x in (self.center, self.direction, self.lo, self.hi)):
            raise ValueError('all per-servo parameter arrays must have one entry per joint')

        self.ser = None
        if not gp('dry_run'):
            self.open_serial(gp('port'), int(gp('baud')))
        if self.ser is None:
            self.get_logger().warn('DRY-RUN: nothing will be sent to hardware')

        self.pos = {j: 0.0 for j in self.joints}                   # last commanded (rad)
        self.plan = [(time.monotonic(), dict(self.pos))]           # [(t_abs, {joint: rad})]
        self.last_line = None

        self.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory',
                                 self.on_traj, 10)
        self.create_timer(1.0 / float(gp('rate_hz')), self.tick)
        self.get_logger().info('hardware_bridge ready, listening on /arm_controller/joint_trajectory')

    def open_serial(self, port, baud):
        if serial is None:
            self.get_logger().error('pyserial missing: pip install pyserial / apt install python3-serial')
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            time.sleep(2.0)  # Arduino resets when the port opens
            self.get_logger().info(f'Opened {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {port}: {e}')
            self.ser = None

    # ---- trajectory handling ----
    def on_traj(self, msg: JointTrajectory):
        if not msg.points:
            return
        now = time.monotonic()
        cur = self.sample(now)
        plan = [(now, dict(cur))]
        prev = dict(cur)
        for pt in msg.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            wp = dict(prev)
            for name, p in zip(msg.joint_names, pt.positions):
                if name in wp:
                    wp[name] = float(p)
            plan.append((now + t, wp))
            prev = wp
        self.plan = plan

    def sample(self, now):
        plan = self.plan
        if now >= plan[-1][0]:
            return dict(plan[-1][1])
        for (t0, a), (t1, b) in zip(plan, plan[1:]):
            if t0 <= now < t1:
                alpha = (now - t0) / (t1 - t0)
                return {j: a[j] + alpha * (b[j] - a[j]) for j in self.joints}
        return dict(plan[0][1])

    # ---- conversion + transport ----
    def rad_to_servo(self, i, rad):
        deg = self.center[i] + self.direction[i] * math.degrees(rad)
        return max(self.lo[i], min(self.hi[i], deg))

    def tick(self):
        self.pos = self.sample(time.monotonic())
        angles = [self.rad_to_servo(i, self.pos[j]) for i, j in enumerate(self.joints)]
        line = 'J,' + ','.join(f'{a:.1f}' for a in angles) + '\n'
        if line == self.last_line:
            return
        self.last_line = line
        if self.ser is not None:
            try:
                self.ser.write(line.encode('ascii'))
            except Exception as e:  # serial.SerialException, OSError
                self.get_logger().error(f'Serial write failed: {e}')
                self.ser = None
        else:
            self.get_logger().debug(line.strip())


def main():
    rclpy.init()
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.ser is not None:
        node.ser.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

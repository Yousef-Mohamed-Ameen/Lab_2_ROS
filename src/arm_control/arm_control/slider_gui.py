#!/usr/bin/env python3
"""Tkinter slider GUI -> /arm_controller/joint_trajectory (same topic for sim, RViz2, hardware)."""
import math
import tkinter as tk

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
LABELS = ['joint1 (base yaw)', 'joint2 (shoulder)', 'joint3 (elbow)', 'joint4 (wrist)']


class SliderGui(Node):
    def __init__(self):
        super().__init__('slider_gui')
        self.declare_parameter('move_time', 0.3)
        self.move_time = float(self.get_parameter('move_time').value)
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.root = tk.Tk()
        self.root.title('Arm joint trajectory publisher')
        self.vars = []
        for label in LABELS:
            row = tk.Frame(self.root)
            row.pack(fill='x', padx=8, pady=4)
            tk.Label(row, text=label, width=18, anchor='w').pack(side='left')
            v = tk.DoubleVar(value=0.0)
            tk.Scale(row, from_=-90, to=90, resolution=1, orient='horizontal', length=380,
                     variable=v, command=lambda _val: self.send()).pack(side='left')
            tk.Label(row, text='deg').pack(side='left')
            self.vars.append(v)
        tk.Button(self.root, text='Home (all 0)', command=self.home).pack(pady=6)

    def home(self):
        for v in self.vars:
            v.set(0.0)
        self.send()

    def send(self):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        p = JointTrajectoryPoint()
        p.positions = [math.radians(v.get()) for v in self.vars]
        p.time_from_start = Duration(sec=int(self.move_time),
                                     nanosec=int((self.move_time % 1) * 1e9))
        msg.points.append(p)
        self.pub.publish(msg)

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(10, self.spin)

    def run(self):
        self.root.after(10, self.spin)
        self.root.mainloop()


def main():
    rclpy.init()
    gui = SliderGui()
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    gui.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

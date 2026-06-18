#!/usr/bin/env python3
"""Interactive single-slider GUI to drive the planetary mixer by hand.

Publishes the sun-joint angle (rad) as std_msgs/Float64 to the absolute topic
/mixer/sun_joint_cmd. planetary_kinematics_node consumes it and drives all six
joints, so dragging this one slider sweeps the whole assembly. Use it to watch
the motion in Gazebo Classic (and/or RViz) and diagnose what looks wrong.

Requires python3-tk (Tkinter) at runtime: sudo apt install python3-tk
"""

import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# Sun-angle slider range: +/-3 sun revolutions ~= +/-1 full shroud revolution
# (shroud = sun / 3), enough to see a complete orbit in either direction.
SLIDER_MIN = -18.85
SLIDER_MAX = 18.85
SLIDER_RES = 0.05
PUBLISH_PERIOD_MS = 40  # ~25 Hz; keeps the kinematics node streaming to Gazebo


class SunJointSliderGui(Node):
    def __init__(self):
        super().__init__('sun_joint_slider_gui')
        self.pub = self.create_publisher(Float64, '/mixer/sun_joint_cmd', 10)
        self.get_logger().info(
            'Sun-joint slider active -> /mixer/sun_joint_cmd '
            f'(range {SLIDER_MIN:.2f} .. {SLIDER_MAX:.2f} rad)'
        )

    def publish(self, angle: float):
        msg = Float64()
        msg.data = float(angle)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SunJointSliderGui()

    root = tk.Tk()
    root.title('Mixer sun-joint slider')

    tk.Label(root, text='sun_joint angle (rad)').pack(padx=12, pady=(12, 0))

    scale = tk.Scale(
        root,
        from_=SLIDER_MIN,
        to=SLIDER_MAX,
        resolution=SLIDER_RES,
        orient=tk.HORIZONTAL,
        length=420,
    )
    scale.set(0.0)
    scale.pack(padx=12, pady=4)

    tk.Button(root, text='Zero', command=lambda: scale.set(0.0)).pack(pady=(0, 12))

    def tick():
        # Publish the current slider value every period (not only on change) so the
        # kinematics node keeps streaming and Gazebo is more likely to repaint.
        node.publish(scale.get())
        rclpy.spin_once(node, timeout_sec=0)
        root.after(PUBLISH_PERIOD_MS, tick)

    def on_close():
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.after(PUBLISH_PERIOD_MS, tick)

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

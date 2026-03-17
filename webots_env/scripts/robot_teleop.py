#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, Float64, Empty
from geometry_msgs.msg import Twist


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        # ----- Head incremental control -----
        self.head_pan_target = 0.0
        self.head_tilt_target = 0.0
        self.head_speed = 0.01
        self.pan_max = 1.57             # ±90°
        self.tilt_max = 0.5             # ±30°

        # ----- Torso control with D‑pad -----
        self.torso_target = 0.3014      # initial position (fully extended)
        self.torso_min = 0.0
        self.torso_max = 0.3014
        self.torso_step = 0.01

        # ----- Base movement -----
        self.speedX = 0.0
        self.speedY = 0.0
        self.yaw = 0.0
        self.stop = False
        self.skip_state = False
        self.once = True

        # ----- Button/axis states -----
        self.prev_left_stick_button = False
        self.pad_vert = 0.0              # D‑pad up/down (axes[7])

        # ----- Subscriber -----
        self.sub_joy = self.create_subscription(Joy, '/hardware/joy', self.joy_callback, 10)

        # ----- Publishers -----
        # Base
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)
        # Head (separate and combined)
        self.pan_pub = self.create_publisher(Float64, '/head/pan_cmd', 10)
        self.tilt_pub = self.create_publisher(Float64, '/head/tilt_cmd', 10)
        self.pub_head = self.create_publisher(Float64MultiArray, '/hardware/head/goal_pose', 10)
        # Torso
        self.pub_torso = self.create_publisher(Float64, '/torso/cmd_pos', 10)
        # Stop / skip
        self.pub_stop = self.create_publisher(Empty, '/stop', 10)
        self.pub_skip = self.create_publisher(Empty, '/hardware/robot_state/skip_state', 10)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Joystick teleop node started")

    def joy_callback(self, msg: Joy):
        # --- Buttons ---
        self.stop = (msg.buttons[1] == 1)
        self.skip_state = (msg.buttons[3] == 1)

        # Left stick button (index 9) for head centering
        left_stick_button = msg.buttons[9] if len(msg.buttons) > 9 else 0
        if left_stick_button == 1 and not self.prev_left_stick_button:
            self.head_pan_target = 0.0
            self.head_tilt_target = 0.0
            #self.get_logger().info("Head centered")
        self.prev_left_stick_button = left_stick_button

        # --- Axes ---
        left_x = msg.axes[0]
        left_y = msg.axes[1]

        # Triggers (axes[2] and axes[5]) – lateral speed
        if msg.axes[2] == 0 and msg.axes[5] != 0:
            left_trigger = msg.axes[2]
            right_trigger = -(msg.axes[5] - 1)
        elif msg.axes[5] == 0 and msg.axes[2] != 0:
            left_trigger = -(msg.axes[2] - 1)
            right_trigger = msg.axes[5]
        elif msg.axes[5] == 0 and msg.axes[2] == 0:
            left_trigger = 0.0
            right_trigger = 0.0
        else:
            left_trigger = -(msg.axes[2] - 1)
            right_trigger = -(msg.axes[5] - 1)

        magnitud_trigger = math.hypot(left_trigger, right_trigger)
        if magnitud_trigger > 0.2:
            self.speedY = (left_trigger - right_trigger) / 4.0
        else:
            self.speedY = 0.0

        # Right stick (axes[3], axes[4]) - base movement
        right_x = msg.axes[3]
        right_y = msg.axes[4]
        magnitud_right = math.hypot(right_x, right_y)
        if magnitud_right > 0.2:
            self.speedX = 0.5 * right_y
            self.yaw = 0.8 * right_x
        else:
            self.speedX = 0.0
            self.yaw = 0.0

        # D‑pad vertical (axes[7]) - torso control
        self.pad_vert = msg.axes[7] if len(msg.axes) > 7 else 0.0

        # --- Incremental head update (left stick) ---
        magnitud_left = math.hypot(left_x, left_y)
        if magnitud_left > 0.2:
            norm_x = left_x / magnitud_left
            norm_y = left_y / magnitud_left
            increment = self.head_speed * magnitud_left
            self.head_pan_target += norm_x * increment
            self.head_tilt_target += norm_y * increment
            # Clamp
            self.head_pan_target = max(min(self.head_pan_target, self.pan_max), -self.pan_max)
            self.head_tilt_target = max(min(self.head_tilt_target, self.tilt_max), -self.tilt_max)

    def timer_callback(self):
        # --- Base twist ---
        if abs(self.speedX) > 0.0 or abs(self.speedY) > 0.0 or abs(self.yaw) > 0.0:
            twist = Twist()
            twist.linear.x = self.speedX
            twist.linear.y = self.speedY / 2.0
            twist.angular.z = self.yaw
            self.pub_twist.publish(twist)
            self.once = True
        else:
            if self.once:
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.pub_twist.publish(twist)
                self.once = False

        # --- Head commands ---
        pan_msg = Float64()
        pan_msg.data = self.head_pan_target
        tilt_msg = Float64()
        tilt_msg.data = self.head_tilt_target
        self.pan_pub.publish(pan_msg)
        self.tilt_pub.publish(tilt_msg)

        head_msg = Float64MultiArray()
        head_msg.data = [self.head_pan_target, self.head_tilt_target]
        self.pub_head.publish(head_msg)

        # --- Torso control (D-pad) ---
        if self.pad_vert > 0.5:
            self.torso_target += self.torso_step
        elif self.pad_vert < -0.5:
            self.torso_target -= self.torso_step
        self.torso_target = max(self.torso_min, min(self.torso_max, self.torso_target))
        torso_msg = Float64()
        torso_msg.data = self.torso_target
        self.pub_torso.publish(torso_msg)

        # --- Stop / skip commands ---
        if self.stop:
            self.pub_stop.publish(Empty())
        if self.skip_state:
            self.pub_skip.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


msg = """
Justina Teleop
---------------------------
Moving around:

        w
   a    s    d

q/e : rotate
x   : stop

CTRL-C to quit
"""


move_bindings = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
}

speed = 0.4
turn = 1.0


def get_key():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):

    def __init__(self):
        super().__init__('justina_teleop')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_cmd(self, x, y, th):
        twist = Twist()

        twist.linear.x = x * speed
        twist.linear.y = y * speed
        twist.angular.z = th * turn

        self.pub.publish(twist)


def main():

    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TeleopNode()

    print(msg)

    x = 0
    y = 0
    th = 0

    try:

        while True:

            key = get_key()

            if key in move_bindings:
                x, y, th = move_bindings[key]

            elif key == 'x':
                x = 0
                y = 0
                th = 0

            else:
                x = 0
                y = 0
                th = 0

                if key == '\x03':
                    break

            node.send_cmd(x, y, th)

    finally:

        twist = Twist()
        node.pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
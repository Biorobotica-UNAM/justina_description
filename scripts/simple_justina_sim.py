#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import TransformBroadcaster


class SimpleJustinaSim(Node):
    """Minimal base simulator for navigation tests in RViz.

    It integrates /hardware/base/cmd_vel, publishes odom -> base_link,
    publishes fake joint states so robot_state_publisher can build the URDF
    tree, and provides an always-clear laser scan.
    """

    def __init__(self):
        super().__init__('simple_justina_sim')

        self.declare_parameter('rate', 30.0)
        self.declare_parameter('initial_x', 6.5)
        self.declare_parameter('initial_y', 3.2)
        self.declare_parameter('initial_yaw', 0.0)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.cmd = Twist()
        self.wheel_positions = [0.0, 0.0, 0.0]

        self.create_subscription(Twist, '/hardware/base/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.initial_pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/navigation/initialpose',
                                 self.initial_pose_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/hardware/base/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/hardware/joint_states', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/hardware/base/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        rate = float(self.get_parameter('rate').value)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Simple Justina simulator ready on /hardware/base/cmd_vel')

    def cmd_vel_callback(self, msg):
        self.cmd = msg

    def initial_pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(orientation.z, orientation.w)
        self.cmd = Twist()
        self.get_logger().info(
            f'Reset pose to x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}')

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        vx = self.cmd.linear.x
        vy = self.cmd.linear.y
        wz = self.cmd.angular.z

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * self.dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * self.dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * self.dt),
                              math.cos(self.yaw + wz * self.dt))

        self.publish_odometry(stamp, vx, vy, wz)
        self.publish_joint_states(stamp, vx, vy, wz)
        self.publish_laser(stamp)

    @staticmethod
    def yaw_to_quaternion(yaw):
        return math.sin(yaw * 0.5), math.cos(yaw * 0.5)

    @staticmethod
    def quaternion_to_yaw(qz, qw):
        return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

    def publish_odometry(self, stamp, vx, vy, wz):
        qz, qw = self.yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def publish_joint_states(self, stamp, vx, vy, wz):
        js = JointState()
        js.header.stamp = stamp
        js.name = [
            'wheel0_joint', 'wheel1_joint', 'wheel2_joint',
            'torso_joint',
            'head_pan_joint', 'head_tilt_joint',
            'lf_shoulder_joint', 'ls_shoulder_joint',
            'l_arm_twist_joint', 'l_elbow_joint',
            'l_wrist_joint', 'l_hand_joint',
            'left_gripper_R_joint', 'left_gripper_L_joint',
            'rf_shoulder_joint', 'rs_shoulder_joint',
            'r_arm_twist_joint', 'r_elbow_joint',
            'r_wrist_joint', 'r_hand_joint',
            'right_gripper_R_joint', 'right_gripper_L_joint',
        ]

        wheel_speed = math.hypot(vx, vy) + abs(wz) * 0.15
        for i in range(3):
            self.wheel_positions[i] += wheel_speed * self.dt

        js.position = self.wheel_positions + [0.301, 0.0, 0.0]
        js.position.extend([0.0] * (len(js.name) - len(js.position)))
        self.joint_pub.publish(js)

    def publish_laser(self, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = math.radians(1.0)
        scan.scan_time = self.dt
        scan.range_min = 0.06
        scan.range_max = 4.0
        scan.ranges = [scan.range_max] * 181
        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = SimpleJustinaSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

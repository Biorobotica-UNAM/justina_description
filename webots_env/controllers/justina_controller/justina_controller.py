#!/usr/bin/env python3
# justina_controller.py
from controller import Robot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
import math

class JustinaController(Node):
    """
    ROS2 controller for 3-wheel omnidirectional base.
    TF: odom -> base_footprint -> base_link
    Publishes /odom, /joint_states, /hardware/base/scan (LaserScan), and /clock.
    """

    def __init__(self, robot: Robot):
        super().__init__('justina_controller')
        # Use simulation time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.robot = robot
        self.time_step = int(robot.getBasicTimeStep())

        # ---------------------------
        # Geometry & odometry state
        # ---------------------------
        self.wheel_radius = 0.06
        self.L = 0.165
        self.Vx = 0.0
        self.Vy = 0.0
        self.w = 0.0
        self.x = 0.0
        self.y = 1.0      # robot spawn at Y=1 (from your world)
        self.theta = 0.0
        self.last_sim_time = None

        # ---------------------------
        # Wheel motors and position sensors
        # ---------------------------
        self.wheels = []
        self.wheel_sensors = []
        wheel_names = ['wheel0_joint', 'wheel1_joint', 'wheel2_joint']
        for name in wheel_names:
            motor = robot.getDevice(name)
            if motor:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
                self.wheels.append(motor)
                sensor = robot.getPositionSensor(name + "_sensor")
                if sensor:
                    sensor.enable(self.time_step)
                    self.wheel_sensors.append(sensor)
                else:
                    self.wheel_sensors.append(None)
                    self.get_logger().warn(f"No position sensor for {name}")
            else:
                self.get_logger().error(f"Motor {name} not found!")

        # ---------------------------
        # All movable joints (for joint_states)
        # ---------------------------
        self.joint_names = [
            'wheel0_joint', 'wheel1_joint', 'wheel2_joint',
            'head_pan_joint', 'head_tilt_joint',
            'lf_shoulder_joint', 'ls_shoulder_joint', 'l_arm_twist_joint', 'l_elbow_joint',
            'l_wrist_joint', 'l_hand_joint', 'left_griper_R_joint', 'left_griper_L_joint',
            'rf_shoulder_joint', 'rs_shoulder_joint', 'r_arm_twist_joint', 'r_elbow_joint',
            'r_wrist_joint', 'r_hand_joint', 'right_griper_R_joint', 'right_griper_L_joint'
        ]

        # store motor devices for later control
        self.other_motors = {}
        for name in self.joint_names[3:]:
            motor = robot.getDevice(name)
            if motor:
                self.other_motors[name] = motor

        # ---------------------------
        # ROS publishers & subscribers
        # ---------------------------
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # LaserScan publisher (topic matches your real robot)
        self.laser_pub = self.create_publisher(LaserScan, '/hardware/base/scan', 10)

        # Clock publisher for simulation time
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        # ---------------------------
        # Lidar initialization
        # ---------------------------
        self.lidar = robot.getDevice('hokuyo_urg_04lx')
        if self.lidar:
            self.lidar.enable(self.time_step)
            self.lidar.enablePointCloud()   # optional, for Webots point cloud view
            self.get_logger().info("Hokuyo URG-04LX lidar initialized.")
        else:
            self.get_logger().error("Could not find lidar device 'hokuyo_urg_04lx'!")

        self.get_logger().info("Controller initialized.")

    def cmd_callback(self, msg: Twist):
        self.Vx = msg.linear.x
        self.Vy = msg.linear.y
        self.w = msg.angular.z

    def step(self) -> bool:
        if self.robot.step(self.time_step) == -1:
            return False

        # Get simulation time from Webots (seconds)
        sim_time_sec = self.robot.getTime()
        sec = int(sim_time_sec)
        nanosec = int((sim_time_sec - sec) * 1e9)
        current_time = Time(sec=sec, nanosec=nanosec)

        # Publish /clock for other nodes
        clock_msg = Clock(clock=current_time)
        self.clock_pub.publish(clock_msg)

        # Compute dt using simulation time
        if self.last_sim_time is None:
            dt = 0.0
        else:
            dt = sim_time_sec - self.last_sim_time
        self.last_sim_time = sim_time_sec

        # 3‑wheel omni inverse kinematics
        sin60 = math.sin(math.pi/3)
        cos60 = math.cos(math.pi/3)

        v0 = ( self.Vy + self.L * self.w ) / self.wheel_radius
        v1 = ( -sin60 * self.Vx - cos60 * self.Vy + self.L * self.w ) / self.wheel_radius
        v2 = (  sin60 * self.Vx - cos60 * self.Vy + self.L * self.w ) / self.wheel_radius

        if len(self.wheels) == 3:
            self.wheels[0].setVelocity(v0)
            self.wheels[1].setVelocity(v1)
            self.wheels[2].setVelocity(v2)

        # Integrate odometry
        dx = (self.Vx * math.cos(self.theta) - self.Vy * math.sin(self.theta)) * dt
        dy = (self.Vx * math.sin(self.theta) + self.Vy * math.cos(self.theta)) * dt
        dtheta = -self.w * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta

        # Publish ROS2 messages with simulation timestamp
        self.publish_odometry(current_time)
        self.publish_joint_states(current_time)
        self.publish_laser_scan(current_time)

        rclpy.spin_once(self, timeout_sec=0.0)
        return True

    def publish_odometry(self, stamp):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        # TF: odom -> base_footprint
        t_odom = TransformStamped()
        t_odom.header.stamp = stamp
        t_odom.header.frame_id = "odom"
        t_odom.child_frame_id = "base_footprint"
        t_odom.transform.translation.x = self.x
        t_odom.transform.translation.y = self.y
        t_odom.transform.rotation.z = qz
        t_odom.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t_odom)

        # TF: base_footprint -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = stamp
        t_base.header.frame_id = "base_footprint"
        t_base.child_frame_id = "base_link"
        t_base.transform.translation.z = 0.06
        t_base.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_base)

    def publish_joint_states(self, stamp):
        js = JointState()
        js.header.stamp = stamp
        js.name = self.joint_names
        js.position = []

        # Wheel positions (from sensors)
        for i, wheel in enumerate(self.wheels):
            if self.wheel_sensors[i] is not None:
                pos = self.wheel_sensors[i].getValue()
            else:
                pos = wheel.getTargetPosition()
                if math.isnan(pos):
                    pos = 0.0
            js.position.append(pos)

        # Other joints – publish 0.0 for now
        for name in self.joint_names[3:]:
            js.position.append(0.0)

        self.joint_pub.publish(js)

    def publish_laser_scan(self, stamp):
        if not self.lidar:
            return

        # Get the range image (list of floats, one per step)
        ranges = self.lidar.getRangeImage()
        if not ranges:
            self.get_logger().warn("Lidar range image is empty.")
            return

        # Create LaserScan message
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser_link"   # must match URDF frame

        # Fixed parameters from URG-04LX specs
        scan.angle_min = -2.094395    # -120 degrees
        scan.angle_max = 2.094395     # +120 degrees
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (len(ranges) - 1)
        scan.time_increment = (1.0 / 10.0) / len(ranges)   # 10 Hz scan
        scan.scan_time = 0.1
        scan.range_min = 0.06
        scan.range_max = 4.1

        # Reverse the order to match ROS convention (angle_min to angle_max)
        scan.ranges = list(reversed([r if r <= scan.range_max else float('inf') for r in ranges]))

        # Publish
        self.laser_pub.publish(scan)

def main():
    rclpy.init()
    robot = Robot()
    controller = JustinaController(robot)
    while controller.step():
        pass
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
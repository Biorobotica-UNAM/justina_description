#!/usr/bin/env python3
from controller import Robot, Supervisor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan, Image, CameraInfo
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
import math
import struct

class JustinaController(Node):
    """
    ROS2 controller for 3-wheel omnidirectional base with torso and arms.
    Publishes /odom, /joint_states, /hardware/base/scan, and Kinect RGB-D.
    Camera frames are provided by the URDF (robot_state_publisher) – no manual transforms.
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
        self.y = 1.0      # robot spawn at Y=1
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
                sensor = robot.getDevice(name + "_sensor")
                if sensor:
                    sensor.enable(self.time_step)
                    self.wheel_sensors.append(sensor)
                else:
                    self.wheel_sensors.append(None)
                    self.get_logger().warn(f"No position sensor for {name}")
            else:
                self.get_logger().error(f"Motor {name} not found!")

        # ---------------------------
        # Torso prismatic joint
        # ---------------------------
        self.torso_motor = robot.getDevice('torso_joint')
        if self.torso_motor:
            # Move torso to maximum height
            self.torso_motor.setPosition(0.3014)   # target position (max) 0.3014
            self.torso_motor.setVelocity(10)       # speed in m/s
            self.torso_sensor = robot.getDevice('torso_joint_sensor')
            if self.torso_sensor:
                self.torso_sensor.enable(self.time_step)
            else:
                self.torso_sensor = None
                self.get_logger().warn("No position sensor for torso_joint")
        else:
            self.get_logger().error("Motor torso_joint not found!")
            self.torso_sensor = None

        # ---------------------------
        # All movable joints (joint_states) – names must match URDF
        # ---------------------------
        self.joint_names = [
            'wheel0_joint', 'wheel1_joint', 'wheel2_joint',
            'torso_joint',
            'head_pan_joint', 'head_tilt_joint',
            'lf_shoulder_joint', 'ls_shoulder_joint', 'l_arm_twist_joint', 'l_elbow_joint',
            'l_wrist_joint', 'l_hand_joint', 'left_griper_R_joint', 'left_griper_L_joint',
            'rf_shoulder_joint', 'rs_shoulder_joint', 'r_arm_twist_joint', 'r_elbow_joint',
            'r_wrist_joint', 'r_hand_joint', 'right_griper_R_joint', 'right_griper_L_joint'
        ]

        # store motor devices for later control (optional, not used now)
        self.other_motors = {}
        for name in self.joint_names[4:]:   # skip wheels and torso
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

        # LaserScan publisher
        self.laser_pub = self.create_publisher(LaserScan, '/hardware/base/scan', 10)

        # Clock publisher
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        # ---------------------------
        # Lidar (Hokuyo) initialization
        # ---------------------------
        hokuyo_name = 'URG-04LX'  # match the name in Webots robot
        self.lidar = robot.getDevice(hokuyo_name)
        if self.lidar:
            self.lidar.enable(self.time_step)
            self.lidar.enablePointCloud()
            self.get_logger().info("Hokuyo '%s' lidar initialized." % hokuyo_name)
        else:
            self.get_logger().error("Could not find lidar device '%s'!" % hokuyo_name)

        # ---------------------------
        # Kinect initialization
        # ---------------------------
        kinect_name_prefix = "camera"  # must match cameraName in webots robot
        self.kinect_color = robot.getDevice(kinect_name_prefix + "_color")
        self.kinect_depth = robot.getDevice(kinect_name_prefix + "_range")

        if self.kinect_color:
            self.kinect_color.enable(self.time_step)
            self.color_width = self.kinect_color.getWidth()
            self.color_height = self.kinect_color.getHeight()
            self.get_logger().info(f"Kinect color camera enabled: {self.color_width}x{self.color_height}")
        else:
            self.get_logger().error(f"Could not find Kinect color device '{kinect_name_prefix} color'!")

        if self.kinect_depth:
            self.kinect_depth.enable(self.time_step)
            self.depth_width = self.kinect_depth.getWidth()
            self.depth_height = self.kinect_depth.getHeight()
            self.depth_max_range = self.kinect_depth.getMaxRange()
            self.get_logger().info(f"Kinect depth sensor enabled: {self.depth_width}x{self.depth_height}, max range {self.depth_max_range}")
        else:
            self.get_logger().error(f"Could not find Kinect depth device '{kinect_name_prefix} range'!")

        # Publishers for Kinect data
        self.color_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

        # Camera info (same for color and depth)
        if self.kinect_color or self.kinect_depth:
            fov = 1.0
            w = self.color_width if self.kinect_color else self.depth_width
            h = self.color_height if self.kinect_color else self.depth_height
            fx = w / (2.0 * math.tan(fov / 2.0))
            fy = fx
            cx = (w - 1) / 2.0
            cy = (h - 1) / 2.0
            self.camera_info = CameraInfo()
            self.camera_info.width = w
            self.camera_info.height = h
            self.camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            self.camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        else:
            self.camera_info = None

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
        self.publish_kinect_data(current_time)
        # No manual transforms for Kinect – they come from URDF

        rclpy.spin_once(self, timeout_sec=0.001)
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

        # Get actual robot height from Webots (Supervisor needed)
        actual_pos = self.robot.getSelf().getPosition()   # [x, y, z] in world coordinates
        actual_z = actual_pos[2]

        # TF: base_footprint -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = stamp
        t_base.header.frame_id = "base_footprint"
        t_base.child_frame_id = "base_link"
        t_base.transform.translation.x = 0.0
        t_base.transform.translation.y = 0.0
        t_base.transform.translation.z = actual_z
        t_base.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_base)

    def publish_joint_states(self, stamp):
        js = JointState()
        js.header.stamp = stamp
        js.name = self.joint_names
        js.position = []

        # Wheel positions (3 joints)
        for i, wheel in enumerate(self.wheels):
            if self.wheel_sensors[i] is not None:
                pos = self.wheel_sensors[i].getValue()
            else:
                pos = wheel.getTargetPosition()
                if math.isnan(pos):
                    pos = 0.0
            js.position.append(pos)

        # Torso position (index 3)
        if self.torso_sensor is not None:
            pos = self.torso_sensor.getValue()
        else:
            pos = self.torso_motor.getTargetPosition() if self.torso_motor else 0.0
            if math.isnan(pos):
                pos = 0.0
        js.position.append(pos)

        # Other joints (arms, head, grippers) – set to 0.0 for now
        for name in self.joint_names[4:]:
            js.position.append(0.0)

        self.joint_pub.publish(js)

    def publish_laser_scan(self, stamp):
        if not self.lidar:
            return

        ranges = self.lidar.getRangeImage()
        if not ranges:
            self.get_logger().warn("Lidar range image is empty.")
            return

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser_link"

        scan.angle_min = -2.094395    # -120 degrees
        scan.angle_max = 2.094395     # +120 degrees
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (len(ranges) - 1)
        scan.time_increment = (1.0 / 10.0) / len(ranges)   # 10 Hz scan
        scan.scan_time = 0.1
        scan.range_min = 0.06
        scan.range_max = 4.1

        # Reverse order to match ROS convention
        scan.ranges = list(reversed([r if r <= scan.range_max else float('inf') for r in ranges]))
        self.laser_pub.publish(scan)

    def publish_kinect_data(self, stamp):
        # Color image
        if self.kinect_color:
            img_data = self.kinect_color.getImage()  # bytes (BGRA)
            if img_data:
                img_msg = Image()
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = "camera_link"
                img_msg.height = self.color_height
                img_msg.width = self.color_width
                img_msg.encoding = "bgra8"
                img_msg.is_bigendian = False
                img_msg.step = self.color_width * 4
                img_msg.data = img_data
                self.color_pub.publish(img_msg)

                if self.camera_info:
                    info_msg = self.camera_info
                    info_msg.header.stamp = stamp
                    info_msg.header.frame_id = "camera_link"
                    self.color_info_pub.publish(info_msg)

        # Depth image
        if self.kinect_depth:
            depth_data = self.kinect_depth.getRangeImage()  # list of floats
            if depth_data:
                depth_msg = Image()
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = "camera_link"
                depth_msg.height = self.depth_height
                depth_msg.width = self.depth_width
                depth_msg.encoding = "32FC1"
                depth_msg.is_bigendian = False
                depth_msg.step = self.depth_width * 4
                depth_msg.data = struct.pack('{}f'.format(len(depth_data)), *depth_data)
                self.depth_pub.publish(depth_msg)

                if self.camera_info:
                    info_msg = self.camera_info
                    info_msg.header.stamp = stamp
                    info_msg.header.frame_id = "camera_link"
                    self.depth_info_pub.publish(info_msg)

def main():
    rclpy.init()
    #robot = Robot()
    robot = Supervisor()
    controller = JustinaController(robot)
    while controller.step():
        pass
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
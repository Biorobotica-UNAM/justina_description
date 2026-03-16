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

    def __init__(self, robot: Robot):
        super().__init__('justina_controller')

        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time',
            rclpy.Parameter.Type.BOOL, True)
        ])

        self.robot = robot
        self.time_step = int(robot.getBasicTimeStep())

        # Robot geometry
        self.wheel_radius = 0.06
        self.L = 0.165

        # Robotino-style speed control
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_w = 0.0

        self.actual_vx = 0.0
        self.actual_vy = 0.0
        self.actual_w = 0.0

        # acceleration limits (Robotino inspired)
        self.max_acc = [10.0, 6.0, 20.0]

        # odometry
        self.x = 0.0
        self.y = 1.0
        self.theta = 0.0
        self.last_sim_time = None

        # wheels
        self.wheels = []
        self.wheel_sensors = []

        wheel_names = ['wheel0_joint','wheel1_joint','wheel2_joint']

        for name in wheel_names:

            motor = robot.getDevice(name)
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

            self.wheels.append(motor)

            sensor = robot.getDevice(name + "_sensor")
            if sensor:
                sensor.enable(self.time_step)
                self.wheel_sensors.append(sensor)
            else:
                self.wheel_sensors.append(None)

        # torso
        self.torso_motor = robot.getDevice('torso_joint')

        if self.torso_motor:
            self.torso_motor.setPosition(0.3014)
            self.torso_motor.setVelocity(10)

            self.torso_sensor = robot.getDevice('torso_joint_sensor')

            if self.torso_sensor:
                self.torso_sensor.enable(self.time_step)
        else:
            self.torso_sensor = None

        # ROS
        self.create_subscription(Twist,'/cmd_vel',self.cmd_callback,10)

        self.odom_pub = self.create_publisher(Odometry,'/odom',10)
        self.joint_pub = self.create_publisher(JointState,'/joint_states',10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.laser_pub = self.create_publisher(LaserScan,'/hardware/base/scan',10)
        self.clock_pub = self.create_publisher(Clock,'/clock',10)

        # Hokuyo
        self.lidar = robot.getDevice('URG-04LX')
        if self.lidar:
            self.lidar.enable(self.time_step)
            self.lidar.enablePointCloud()

        # Kinect
        self.kinect_color = robot.getDevice("camera_color")
        self.kinect_depth = robot.getDevice("camera_range")

        if self.kinect_color:
            self.kinect_color.enable(self.time_step)
            self.color_width = self.kinect_color.getWidth()
            self.color_height = self.kinect_color.getHeight()

        if self.kinect_depth:
            self.kinect_depth.enable(self.time_step)
            self.depth_width = self.kinect_depth.getWidth()
            self.depth_height = self.kinect_depth.getHeight()

        self.color_pub = self.create_publisher(Image,'/camera/image_raw',10)
        self.depth_pub = self.create_publisher(Image,'/camera/depth/image_raw',10)

        self.get_logger().info("Justina controller initialized")

    # ----------------------------------
    # cmd_vel callback
    # ----------------------------------

    def cmd_callback(self,msg):

        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_w = msg.angular.z

    # ----------------------------------
    # Robotino acceleration model
    # ----------------------------------

    def accelerate(self):

        dt = self.time_step / 1000.0

        target = [self.target_vx,self.target_vy,self.target_w]
        actual = [self.actual_vx,self.actual_vy,self.actual_w]

        max_steps = 0

        for i in range(3):
            steps = abs(target[i] - actual[i])
            steps /= self.max_acc[i] * dt
            if steps > max_steps:
                max_steps = steps

        if max_steps < 1:
            max_steps = 1

        for i in range(3):
            actual[i] += (target[i] - actual[i]) / max_steps

        self.actual_vx = actual[0]
        self.actual_vy = actual[1]
        self.actual_w = actual[2]

    # ----------------------------------
    # Main step
    # ----------------------------------

    def step(self):

        if self.robot.step(self.time_step) == -1:
            return False

        sim_time = self.robot.getTime()

        sec = int(sim_time)
        nanosec = int((sim_time-sec)*1e9)

        stamp = Time(sec=sec,nanosec=nanosec)

        self.clock_pub.publish(Clock(clock=stamp))

        if self.last_sim_time is None:
            dt = 0.0
        else:
            dt = sim_time - self.last_sim_time

        self.last_sim_time = sim_time

        # smooth acceleration
        self.accelerate()

        vx = self.actual_vx
        vy = self.actual_vy
        w = self.actual_w

        sin60 = math.sin(math.pi/3)
        cos60 = math.cos(math.pi/3)

        v0 = (vy + self.L*w)/self.wheel_radius
        v1 = (-sin60*vx - cos60*vy + self.L*w)/self.wheel_radius
        v2 = ( sin60*vx - cos60*vy + self.L*w)/self.wheel_radius

        MAX_WHEEL_SPEED = 12

        v0 = max(-MAX_WHEEL_SPEED,min(MAX_WHEEL_SPEED,v0))
        v1 = max(-MAX_WHEEL_SPEED,min(MAX_WHEEL_SPEED,v1))
        v2 = max(-MAX_WHEEL_SPEED,min(MAX_WHEEL_SPEED,v2))

        self.wheels[0].setVelocity(v0)
        self.wheels[1].setVelocity(v1)
        self.wheels[2].setVelocity(v2)

        # odometry integration
        dx = (vx*math.cos(self.theta)-vy*math.sin(self.theta))*dt
        dy = (vx*math.sin(self.theta)+vy*math.cos(self.theta))*dt

        self.x += dx
        self.y += dy
        self.theta -= w*dt

        self.publish_odometry(stamp)
        self.publish_joint_states(stamp)
        self.publish_laser_scan(stamp)

        rclpy.spin_once(self,timeout_sec=0)

        return True

    # ----------------------------------

    def publish_odometry(self,stamp):

        qz = math.sin(self.theta/2)
        qw = math.cos(self.theta/2)

        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom)

        tf = TransformStamped()

        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)

    # ----------------------------------

    def publish_joint_states(self, stamp):

        js = JointState()
        js.header.stamp = stamp

        js.name = [
            'wheel0_joint', 'wheel1_joint', 'wheel2_joint',
            'torso_joint',

            'head_pan_joint', 'head_tilt_joint',

            'lf_shoulder_joint', 'ls_shoulder_joint',
            'l_arm_twist_joint', 'l_elbow_joint',
            'l_wrist_joint', 'l_hand_joint',
            'left_griper_R_joint', 'left_griper_L_joint',

            'rf_shoulder_joint', 'rs_shoulder_joint',
            'r_arm_twist_joint', 'r_elbow_joint',
            'r_wrist_joint', 'r_hand_joint',
            'right_griper_R_joint', 'right_griper_L_joint'
        ]

        js.position = []

        # Wheels
        for i in range(3):
            if self.wheel_sensors[i]:
                js.position.append(self.wheel_sensors[i].getValue())
            else:
                js.position.append(0.0)

        # Torso
        if self.torso_sensor:
            js.position.append(self.torso_sensor.getValue())
        else:
            js.position.append(0.0)

        # All other joints (not actuated yet)
        for _ in range(len(js.name) - 4):
            js.position.append(0.0)

        self.joint_pub.publish(js)
        
    # ----------------------------------

    def publish_laser_scan(self,stamp):

        if not self.lidar:
            return

        ranges = self.lidar.getRangeImage()

        scan = LaserScan()

        scan.header.stamp = stamp
        scan.header.frame_id = "laser_link"

        scan.angle_min = -2.094395
        scan.angle_max = 2.094395
        scan.angle_increment = (scan.angle_max-scan.angle_min)/(len(ranges)-1)

        scan.range_min = 0.06
        scan.range_max = 4.1

        scan.ranges = list(reversed(ranges))

        self.laser_pub.publish(scan)


def main():

    rclpy.init()

    robot = Supervisor()

    controller = JustinaController(robot)

    while controller.step():
        pass

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
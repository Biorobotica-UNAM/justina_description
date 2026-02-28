# justina_controller.py
from controller import Robot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math

class JustinaController(Node):
    """
    ROS2 controller for 3-wheel omnidirectional base.
    TF: odom -> base_footprint -> base_link
    """

    def __init__(self, robot: Robot):
        super().__init__('justina_controller')

        self.robot = robot
        self.time_step = int(robot.getBasicTimeStep())

        # Geometry
        self.wheel_radius = 0.06  # corrected radius 6cm
        self.L = 0.165  # distance from center to wheel

        # Wheel motors
        self.wheels = [
            robot.getDevice('wheel0_joint'),
            robot.getDevice('wheel1_joint'),
            robot.getDevice('wheel2_joint')
        ]
        for i, wheel in enumerate(self.wheels):
            if wheel is None:
                self.get_logger().error(f"Wheel {i} motor not found!")
            else:
                self.get_logger().info(f"Wheel {i} motor found: {wheel.getName()}")
                wheel.setPosition(float('inf'))
                wheel.setVelocity(0.0)

        # State
        self.Vx = 0.0
        self.Vy = 0.0
        self.w = 0.0
        self.x = 0.0
        self.y = 1.0  # robot spawn at Y=1
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ROS2 interfaces
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Additional actuated joints
        self.actuated_joints = {}

    def cmd_callback(self, msg: Twist):
        self.Vx = msg.linear.x
        self.Vy = msg.linear.y
        self.w = msg.angular.z

    def step(self) -> bool:
        if self.robot.step(self.time_step) == -1:
            return False

        # Time delta
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # 3-wheel omni kinematics
        # v0 = (-math.sin(math.pi) * self.Vx + math.cos(math.pi) * self.Vy + self.L * self.w) / self.wheel_radius
        # v1 = (-math.sin(-math.pi/3) * self.Vx + math.cos(-math.pi/3) * self.Vy + self.L * self.w) / self.wheel_radius
        # v2 = (-math.sin(math.pi/3) * self.Vx + math.cos(math.pi/3) * self.Vy + self.L * self.w) / self.wheel_radius

        sin60 = math.sin(math.pi/3)   # 0.8660254
        cos60 = math.cos(math.pi/3)   # 0.5

        v0 = ( self.Vy + self.L * self.w ) / self.wheel_radius
        v1 = ( -sin60 * self.Vx - cos60 * self.Vy + self.L * self.w ) / self.wheel_radius
        v2 = (  sin60 * self.Vx - cos60 * self.Vy + self.L * self.w ) / self.wheel_radius

        self.wheels[0].setVelocity(v0)
        self.wheels[1].setVelocity(v1)
        self.wheels[2].setVelocity(v2)

        # Integrate odometry (simplified)
        dx = (self.Vx * math.cos(self.theta) - self.Vy * math.sin(self.theta)) * dt
        dy = (self.Vx * math.sin(self.theta) + self.Vy * math.cos(self.theta)) * dt
        dtheta = self.w * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta

        self.get_logger().info(f"cmd_vel: Vx={self.Vx}, Vy={self.Vy}, w={self.w} -> v0={v0}, v1={v1}, v2={v2}")
        
        # Publish ROS2 messages
        self.publish_odometry(now)
        self.publish_joint_states(now)
        rclpy.spin_once(self, timeout_sec=0.0)
        return True

    def publish_odometry(self, now):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        # TF: odom -> base_footprint
        t_odom = TransformStamped()
        t_odom.header.stamp = now.to_msg()
        t_odom.header.frame_id = "odom"
        t_odom.child_frame_id = "base_footprint"
        t_odom.transform.translation.x = self.x
        t_odom.transform.translation.y = self.y
        t_odom.transform.rotation.z = qz
        t_odom.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t_odom)

        # TF: base_footprint -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = now.to_msg()
        t_base.header.frame_id = "base_footprint"
        t_base.child_frame_id = "base_link"
        t_base.transform.translation.z = 0.06
        t_base.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_base)

    def publish_joint_states(self, now):
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['wheel0_joint', 'wheel1_joint', 'wheel2_joint']
        js.position = [w.getTargetPosition() for w in self.wheels]

        for joint_name, device in self.actuated_joints.items():
            js.name.append(joint_name)
            js.position.append(device.getTargetPosition() if device else 0.0)

        self.joint_pub.publish(js)

    def add_joint(self, joint_name: str, device):
        self.actuated_joints[joint_name] = device

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
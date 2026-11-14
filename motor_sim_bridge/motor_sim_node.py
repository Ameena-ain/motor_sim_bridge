import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from motor_sim_bridge.motor_model import cmd_vel_to_rpm, motor_step


class MotorSimNode(Node):
    def __init__(self):
        super().__init__('motor_sim_node')
        self.r = 0.05
        self.L = 0.30
        self.tau = 0.3
        self.dt = 0.02

        self.target_left = 0.0
        self.target_right = 0.0
        self.left_rpm = 0.0
        self.right_rpm = 0.0

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.pub = self.create_publisher(Float32MultiArray, 'motor/rpm_telemetry', 10)
        self.create_timer(self.dt, self.on_timer)

        self.timeout = 0.5
        self.last_cmd_time = self.get_clock().now()

    def on_cmd_vel(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        self.target_left, self.target_right = cmd_vel_to_rpm(v, w, self.r, self.L)
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.timeout:
            self.target_left = 0.0
            self.target_right = 0.0

        self.left_rpm = motor_step(self.left_rpm, self.target_left, self.dt, self.tau)
        self.right_rpm = motor_step(self.right_rpm, self.target_right, self.dt, self.tau)
        out = Float32MultiArray()
        out.data = [self.left_rpm, self.right_rpm]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = MotorSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

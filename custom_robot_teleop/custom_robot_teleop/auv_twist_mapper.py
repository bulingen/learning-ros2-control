import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Float64


class CmdVelToSingleArray(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_single_array")

        self.declare_parameter("in", "/cmd_vel")
        self.declare_parameter("propellers", "/propeller_controller/commands")
        self.declare_parameter("thrust", "/cmd_thrust")

        in_topic = self.get_parameter("in").get_parameter_value().string_value
        propellers_topic = (
            self.get_parameter("propellers").get_parameter_value().string_value
        )
        thrust_topic = self.get_parameter("thrust").get_parameter_value().string_value

        self.get_logger().info(f"Subscribing to: {in_topic}")
        self.get_logger().info(f"Publishing to: {propellers_topic}")

        self.propeller_publisher_ = self.create_publisher(
            Float64MultiArray, propellers_topic, 10
        )
        self.thrust_publisher_ = self.create_publisher(Float64, thrust_topic, 10)
        self.subscription = self.create_subscription(
            Twist, in_topic, self.cmd_vel_callback, 10
        )

    def cmd_vel_callback(self, msg: Twist):
        thrust = msg.linear.x
        array_msg = Float64MultiArray()
        array_msg.data = [thrust]
        self.propeller_publisher_.publish(array_msg)

        float_msg = Float64()
        float_msg.data = thrust
        self.thrust_publisher_.publish(float_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSingleArray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

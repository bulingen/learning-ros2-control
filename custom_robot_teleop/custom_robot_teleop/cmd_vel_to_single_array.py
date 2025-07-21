import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelToSingleArray(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_single_array")

        self.declare_parameter("in", "/cmd_vel")
        self.declare_parameter("out", "/velocity_controller/commands")

        in_topic = self.get_parameter("in").get_parameter_value().string_value
        out_topic = self.get_parameter("out").get_parameter_value().string_value

        self.get_logger().info(f"Subscribing to: {in_topic}")
        self.get_logger().info(f"Publishing to: {out_topic}")

        self.publisher_ = self.create_publisher(Float64MultiArray, out_topic, 10)
        self.subscription = self.create_subscription(
            Twist, in_topic, self.cmd_vel_callback, 10
        )

    def cmd_vel_callback(self, msg: Twist):
        thrust = msg.linear.x
        array_msg = Float64MultiArray()
        array_msg.data = [thrust]
        self.publisher_.publish(array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSingleArray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

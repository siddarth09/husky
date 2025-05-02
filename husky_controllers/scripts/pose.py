#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        degree_angles = [
                -15, -45, 90,   # RF: HAA, HFE, KFE
                15,  45, -90,  # LF: HAA, HFE, KFE
                -15, -45, 90,   # RH: HAA, HFE, KFE
                15,  45, -90   # LH: HAA, HFE, KFE
            ]


        msg = Float64MultiArray()
        msg.data = [math.radians(deg) for deg in degree_angles]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published radians: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
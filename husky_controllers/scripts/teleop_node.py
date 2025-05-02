#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import sys, select, termios, tty
from tf_transformations import quaternion_from_euler

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_pub = self.create_publisher(Pose, '/body_pose', 10)

        self.speed = 0.2
        self.turn_speed = 0.5
        self.height = 0.2
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.status_counter = 0
        self.print_interval = 10  # change this to control how often it logs speed/turn

        self.pose_msg = Pose()
        self.settings = termios.tcgetattr(sys.stdin)

        self.timer = self.create_timer(0.1, self.publish_pose)
        self.key_loop()

    def publish_pose(self):
        self.pose_msg.position.z = self.height
        quat = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        self.pose_msg.orientation.x = quat[0]
        self.pose_msg.orientation.y = quat[1]
        self.pose_msg.orientation.z = quat[2]
        self.pose_msg.orientation.w = quat[3]
        self.pose_pub.publish(self.pose_msg)
    def print_instructions(self):
            if self.speed == 0.0 and self.turn_speed == 0.0:
                return

            print(r"""
        🐾 Custom Quadruped Teleop Node (ROS 2)
        ---------------------------------------
        Use your keyboard to send velocity and pose commands to the robot.

        Movement (via /cmd_vel):
        w    : forward
        s    : backward
        a    : rotate left
        d    : rotate right

        Posture (via /body_pose):
        z    : sit (lower body)
        x    : stand (raise body)
        r/f  : pitch forward/backward
        q/e  : roll left/right
        c    : neutral pose (reset body orientation & height)

        Other:
        CTRL+C to exit

        🔧 Initial Settings:
        Linear speed  = {:.2f}
        Turn speed    = {:.2f}
        """.format(self.speed, self.turn_speed))


    def key_loop(self):
        self.print_instructions()
        self.get_logger().info("Press 'CTRL+C' to exit.")

        # Counter to limit log spam
        status_counter = 0
        print_interval = 5

        try:
            while rclpy.ok():
                key = self.get_key()
                twist = Twist()
                should_publish = True

                # Movement
                if key == 'w':
                    twist.linear.x = self.speed
                elif key == 's':
                    twist.linear.x = -self.speed
                elif key == 'a':
                    twist.angular.z = self.turn_speed
                elif key == 'd':
                    twist.angular.z = -self.turn_speed

                # Posture
                elif key == 'z':
                    self.height = 0.05
                    self.get_logger().info("🪑 Sit")
                elif key == 'x':
                    self.height = 0.2
                    self.get_logger().info("🧍 Stand")
                elif key == 'r':
                    self.pitch += 0.05
                elif key == 'f':
                    self.pitch -= 0.05
                elif key == 'q':
                    self.roll += 0.05
                elif key == 'e':
                    self.roll -= 0.05
                elif key == 'c':
                    self.roll = self.pitch = self.yaw = 0.0
                    self.height = 0.2
                    self.get_logger().info("🔄 Neutral pose")

                # Speed adjustment
                elif key == '+':
                    self.speed *= 1.1
                    self.turn_speed *= 1.1
                    status_counter += 1
                elif key == '-':
                    self.speed *= 0.9
                    self.turn_speed *= 0.9
                    status_counter += 1

                # Exit
                elif key == '\x03':
                    break
                else:
                    should_publish = False  # Invalid key — don't send anything

                if should_publish:
                    self.vel_pub.publish(twist)

                    # Log twist only if movement is non-zero
                    if twist.linear.x != 0.0 or twist.angular.z != 0.0:
                        self.get_logger().info(
                            f"Twist → linear: [{twist.linear.x:.2f}] angular: [{twist.angular.z:.2f}]"
                        )

                # Occasionally log current speed/turn
                if status_counter % print_interval == 0 and status_counter > 0:
                    self.get_logger().info(
                        f"⚙️ Speed: {self.speed:.2f} | Turn: {self.turn_speed:.2f}"
                    )

        finally:
            # Stop the robot on exit
            self.vel_pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

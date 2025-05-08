#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros2_ugv.rosmaster import Rosmaster
from ros2_ugv.motion_controller import MotionController

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        self.rosmaster = Rosmaster(com='/dev/myserial', debug=False)
        self.motion = MotionController(self.rosmaster)

        self.rosmaster.create_receive_threading()
        self.rosmaster.set_auto_report_state(True)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("MotionNode started and listening to /cmd_vel")

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x    # forward/backward
        vy = msg.linear.y    # strafe left/right
        omega = msg.angular.z  # yaw rotation
        self.motion.set_velocity(vx, vy, omega)

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.motion.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

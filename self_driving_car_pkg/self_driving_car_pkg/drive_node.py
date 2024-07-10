#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



class DriveNode(Node):

    def __init__(self):
        super().__init__('drive_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Publishing: cmd_vel ')
        self.cmd_vel_msg = Twist()


    def timer_callback(self):
        self.cmd_vel_msg.linear.x = 10.0;
        self.cmd_vel_msg.angular.z = 0.0;

        self.publisher_.publish(self.cmd_vel_msg)



def main(args=None):
    rclpy.init(args=args)

    cmd_vel_publisher = DriveNode()

    rclpy.spin(cmd_vel_publisher)

    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
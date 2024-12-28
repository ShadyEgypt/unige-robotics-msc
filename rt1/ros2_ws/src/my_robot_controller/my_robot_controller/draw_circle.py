#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def timer_callback(self):
        self.get_logger().info("Velocity Updated!")
        self.send_velocity_command(2.0, 1.0)
        
    def send_velocity_command(self, linear_vel, angular_vel):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd_vel)
    
def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()
#!/usr/bin/env python


import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.clock import Clock

from geometry_msgs.msg import PoseStamped

class Spoofer(Node):

    def __init__(self):
        super().__init__('spoofer')

        self.declare_parameter('center',[0.0, 0.0, 0.0])
        self.declare_parameter('period', 0.005)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('omega',0.5)

        self.center = self.get_parameter('center').value
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.period = self.get_parameter('period').value
        self.theta = 0.0

        self.pose_pub = self.create_publisher(PoseStamped, '/uav_1/vehicle_interface/position_setpoint',10)
        self.pub_timer = self.create_timer(self.period,self.publish_spoofed_pose)

    def publish_spoofed_pose(self):
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.center[0] + self.radius*np.cos(self.theta)
        msg.pose.position.y = self.center[1] + self.radius*np.sin(self.theta)
        msg.pose.position.z = self.center[2]
        msg.pose.orientation.w = np.cos(self.theta/2)
        msg.pose.orientation.z = np.sin(self.theta/2)

        self.pose_pub.publish(msg)
        self.theta += self.omega*self.period

def main(args=None):
    rclpy.init(args=args)
    spoofer = Spoofer()
    rclpy.spin(spoofer)
    spoofer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
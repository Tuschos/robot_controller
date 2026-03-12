#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SlaveControllerNode(Node):
    def __init__(self):
        super().__init__('slave_controller')

        # He so dieu khien
        self.k_g = [8.0, 16.0]

        # Thong so master & slave
        self.v = 0.0
        self.omega = 0.0
        self.a_v = 0.0
        self.a_omega = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0

        # Luc phan hoi ao va luc dieu khien robot
        self.f_v = 0.0
        self.f_s = [0.0, 0.0]

        self.master_subscription = self.create_subscription(
            DynamicJointState,
            '/delayed/fd/dynamic_joint_states',
            self.master_callback,
            10
        )

        self.fv_subscription = self.create_subscription(
            Float64,
            '/fictitious_force',
            self.fv_callback,
            10
        )

        self.cmd_vel_pubblisher = self.create_publisher(
            Twist,
            '/diff_drive_controller/cmd_vel_unstamped',
            10
        )

        # Timer dieu khien tan so 100hz
        self.timer_ = self.create_timer(0.01,self.timer_callback)

    def master_callback(self, msg : DynamicJointState):
        self.pos_x = -msg.interface_values[0].values[0]
        self.pos_y = -msg.interface_values[1].values[0]

    def fv_callback(self, msg : Float64):
        self.f_v = msg.data

    def timer_callback(self):
        cmd_vel = Twist()

        cmd_vel.linear.x = self.k_g[0] * self.pos_x - self.f_v
        cmd_vel.angular.z = self.k_g[1] * self.pos_y 

        # Publish van toc robot 
        self.cmd_vel_pubblisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = SlaveControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

class MasterControllerNode(Node):
    def __init__(self):
        super().__init__('master_controller')

        # He so dieu khien
        self.k_m = [10.0, 3.0]
        self.k_g = [9.0, 16.0]
        self.alpha_m = [0.0, 0.0]
        #self.alpha_m = [18.0, 5.0]
        #self.k_p = [0.01, 0.01]
        self.k_p = [0.0, 0.0]

        # Thong so master & slave
        self.v = 0.0
        self.omega = 0.0
        self.pos_x = 0.0
        self.vel_x = 0.0
        self.pos_y = 0.0
        self.vel_y = 0.0
        self.f_m = [0.0, 0.0, 0.0]

        self.master_subscription = self.create_subscription(
            DynamicJointState,
            '/fd/dynamic_joint_states',
            self.master_callback,
            10
        )

        self.slave_subscription = self.create_subscription(
            Odometry,
            '/diff_cont/odom_delayed',
            self.slave_callback,
            10
        )

        self.force_master_publisher = self.create_publisher(
            Float64MultiArray,
            '/fd/fd_controller/commands',
            10
        )

        # Timer dieu khien tan so 100hz
        self.timer_ = self.create_timer(0.01, self.timer_callback)

    def master_callback(self, msg : DynamicJointState):
        self.pos_x = msg.interface_values[0].values[0]
        self.vel_x = msg.interface_values[0].values[1]
        self.pos_y = msg.interface_values[1].values[0]
        self.vel_y = msg.interface_values[1].values[1] 

        # if self.pos_x > 0.00:
        #     self.pos_x += 0.015
        # elif self.pos_x < -0.00:
        #     self.pos_x -= 0.015

    def slave_callback(self, msg : Odometry):
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def timer_callback(self):
        force_command = Float64MultiArray()

        # Master position & velocity
        pos_x = -self.pos_x
        vel_x = -self.vel_x
        pos_y = -self.pos_y
        vel_y = -self.vel_y

        # Robot velocities
        v = self.v
        omega = self.omega

        # He so dieu khien
        k_m = self.k_m
        k_g = self.k_g
        alpha_m = self.alpha_m
        k_p = self.k_p

        # Tinh luc phan hoi tren tung truc
        f_x = -k_m[0] * (k_g[0] * pos_x - v) - alpha_m[0] * vel_x - k_p[0] * vel_x
        f_y = -k_m[1] * (k_g[1] * pos_y - omega) - alpha_m[1] * vel_y - k_p[1] * vel_y

        # Gán vào mảng dữ liệu
        self.f_m = [-f_x, -f_y, 0.0]  # trục z = 0

        force_command.data = self.f_m
        self.force_master_publisher.publish(force_command)

        # Log
        #self.get_logger().info(f'velx={vel_x:.3f}, vel_y={vel_y:.3f}, v={v:.2f}, omega={omega:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MasterControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

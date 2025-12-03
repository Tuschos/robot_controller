#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64 
from nav_msgs.msg import Odometry
from control_msgs.msg import DynamicJointState
import csv


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.force_sub = self.create_subscription(Float64MultiArray, '/fd/fd_controller/commands', self.force_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_cb, 10)
        self.fictitious_sub = self.create_subscription(Float64, '/fictitious_force', self.fictitious_cb, 10)
        self.master_sub = self.create_subscription(DynamicJointState, '/fd/dynamic_joint_states_delayed',self.master_cb,10)

        self.csv_file = open('/home/tus/data_log_teleop/delay_damping2.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['time', 'f_x', 'f_y', 'pos_x_robot', 'pos_y_robot', 'f_v', 'pos_x', 'pos_y', 'v', 'omega','Tm2s', "Ts2m"])

        self.f_m = [0.0, 0.0]
        self.pos_robot = [0.0, 0.0]
        self.f_v = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.h1 = 0.0
        self.h2 = 605
        self.start_time = self.get_clock().now().to_msg().sec + 1e-9 * self.get_clock().now().to_msg().nanosec

        self.timer = self.create_timer(0.02, self.log_data)  # 50Hz

    def force_cb(self, msg : Float64MultiArray):
        self.f_m = msg.data[:2]

    def fictitious_cb(self, msg : Float64):
        self.f_v = msg.data

    def odom_cb(self, msg : Odometry):
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z
        self.pos_robot[0] = msg.pose.pose.position.x
        self.pos_robot[1] = msg.pose.pose.position.y

    def master_cb(self, msg : DynamicJointState):
        self.pos_x = msg.interface_values[0].values[0]
        self.pos_y = msg.interface_values[1].values[0]
        self.h1 = self.get_clock().now().to_msg().sec * 1e3 + 1e-6 * self.get_clock().now().to_msg().nanosec - (msg.header.stamp.sec * 1e3 + 1e-6 * msg.header.stamp.nanosec)

    def log_data(self):
        now = self.get_clock().now().to_msg().sec + 1e-9 * self.get_clock().now().to_msg().nanosec - self.start_time
        row = [now, *self.f_m, self.pos_robot[0], self.pos_robot[1], self.f_v, self.pos_x, self.pos_y, self.v, self.omega, self.h1 + self.h2]
        self.writer.writerow(row)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

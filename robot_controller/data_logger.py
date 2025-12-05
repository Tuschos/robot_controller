#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64 
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import csv


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # best effort qos
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # self.force_sub = self.create_subscription(Float64MultiArray, '/fd/fd_controller/commands', self.force_cb, 10)
        self.vel_encoder_sub = self.create_subscription(TwistStamped, '/vel_encoder/data', self.vel_encoder_sub_cb, qos_profile)
        self.fictitious_sub = self.create_subscription(Float64, '/fictitious_force', self.fictitious_cb, 10)
        self.master_sub = self.create_subscription(DynamicJointState, '/fd/dynamic_joint_states',self.master_cb,10)
        # self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_cb, qos_profile)

        self.csv_file = open('/tmp/data_log.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['time', 'pos_x_robot', 'pos_y_robot', 'f_v', 'pos_x_master', 'pos_y_master', 'v', 'omega','Tm2s'])

        self.f_v = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.h1 = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        self.timer = self.create_timer(0.05, self.log_data)  # 20Hz

    def force_cb(self, msg : Float64MultiArray):
        self.f_m = msg.data[:2]

    def fictitious_cb(self, msg : Float64):
        self.f_v = msg.data

    def vel_encoder_sub_cb(self, msg : TwistStamped):
        self.v = msg.twist.linear.x
        self.omega = msg.twist.angular.z

    def master_cb(self, msg : DynamicJointState):
        self.pos_x = msg.interface_values[0].values[0]
        self.pos_y = msg.interface_values[1].values[0]
        self.h1 = self.get_clock().now().nanoseconds * 1e-6 - (msg.header.stamp.sec * 1e3 + 1e-6 * msg.header.stamp.nanosec)

    def odometry_cb(self, msg : Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def log_data(self):
        now = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        row = [now,self.odom_x, self.odom_y, self.f_v, self.pos_x, self.pos_y, self.v, self.omega, self.h1]
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

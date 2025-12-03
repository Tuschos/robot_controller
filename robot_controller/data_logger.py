#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64 
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
import csv


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.force_sub = self.create_subscription(Float64MultiArray, '/fd/fd_controller/commands', self.force_cb, 10)
        self.vel_encoder_sub = self.create_subscription(TwistStamped, '/vel_encoder/data', self.vel_encoder_sub_cb, 10)

        self.csv_file = open('/home/tus/data_log_teleop/master_log.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['time', 'f_x', 'f_y','Ts2m'])

        self.f_m = [0.0, 0.0]
        self.h2 = 0.0
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        self.timer = self.create_timer(0.02, self.log_data)  # 50Hz

    def vel_encoder_sub_cb(self, msg : TwistStamped):
        self.h2 = self.get_clock().now().nanoseconds * 1e-6 -  (msg.header.stamp.sec * 1e3 + 1e-6 * msg.header.stamp.nanosec)

    def force_cb(self, msg : Float64MultiArray):
        self.f_m[0] = msg.data[0]
        self.f_m[1] = msg.data[1]

    def log_data(self):
        now = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        row = [now, *self.f_m, self.h2]
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

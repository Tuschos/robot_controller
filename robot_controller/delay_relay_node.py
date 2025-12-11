#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from collections import deque
import random

from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
import math

class DelayRelayNode(Node):
    def __init__(self):
        super().__init__('delay_relay_node')

        # Thời gian delay
        self.h1 = 0.0
        self.h2 = 0.0
        self.counter = 0

        # Hàng đợi cho từng loại message
        self.queue_master = deque()
        self.queue_odom = deque()

        # Subscriptions
        self.create_subscription(DynamicJointState,
                                 '/fd/dynamic_joint_states',
                                 self.master_callback,
                                 10)

        self.create_subscription(Odometry,
                                 '/diff_cont/odom',
                                 self.odom_callback,
                                 10)

        # Publishers (topic sau delay)
        self.pub_master = self.create_publisher(DynamicJointState,
                                                '/fd/dynamic_joint_states_delayed',
                                                10)

        self.pub_odom = self.create_publisher(Odometry,
                                              '/diff_cont/odom_delayed',
                                              10)


        # Timer xử lý gửi sau delay
        self.create_timer(0.01, self.timer_callback)  # 100 Hz

    def master_callback(self, msg : DynamicJointState):
        now = self.get_clock().now()
        self.queue_master.append((now, msg))

    def odom_callback(self, msg : Odometry):
        now = self.get_clock().now()
        self.queue_odom.append((now, msg))


    def timer_callback(self):

        # #Random time delay
        # self.h2 = 0.2   # 200ms
        # # self.h1 = 0.3 + 0.1 * math.sin(self.counter * 0.1 * 3.14)  # 200-400ms
        # self.counter += 1
        # if( self.counter > 100000000):
        #     self.counter = 0

        # No delay
        # self.h1 = 0.001
        # self.h2 = 0.001

        now = self.get_clock().now()
        
        # Xử lý queue master với fixed delay
        while self.queue_master:
            t, msg = self.queue_master[0]
            if (now - t).nanoseconds * 1e-9 >= self.h1:
                self.pub_master.publish(msg)
                self.queue_master.popleft()
            else:
                break

        # Xử lý queue odom với variable delay
        while self.queue_odom:
            t, msg = self.queue_odom[0]
            if (now - t).nanoseconds * 1e-9 >= self.h2:
                self.pub_odom.publish(msg)
                self.queue_odom.popleft()
            else:
                break


def main(args=None):
    rclpy.init(args=args)
    node = DelayRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
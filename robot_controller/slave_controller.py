#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64 
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

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
        self.f_v = 0.0

        # # Tham so thoi gian, van toc tinh toan gia toc robot
        # self.v_prev = 0.0
        # self.omega_prev = 0.0
        # self.time_prev = self.get_clock().now()
        
        # best effort qos
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.master_subscription = self.create_subscription(
            DynamicJointState,
            '/fd/dynamic_joint_states',
            self.master_callback,
            10
        )

        self.slave_subscription = self.create_subscription(
            TwistStamped,
            '/vel_encoder/data',
            self.slave_callback,
            qos_profile
        )

        self.fv_subscription = self.create_subscription(
            Float64,
            '/fictitious_force',
            self.fv_callback,
            10
        )

        self.cmd_vel_pubblisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer dieu khien tan so 100hz
        self.timer_ = self.create_timer(0.05,self.timer_callback)

    def master_callback(self, msg : DynamicJointState):
        self.pos_x = -msg.interface_values[0].values[0]
        self.pos_y = -msg.interface_values[1].values[0]

        # Anh xa vi tri tay cam khi co gioi han vat ly
        # if self.pos_x > 0.00:
        #     self.pos_x += 0.015
        # elif self.pos_x < -0.00:
        #     self.pos_x -= 0.015
        # else:
        #     if self.pos_y >= 0.01:
        #         self.pos_y += 0.1726
        #     elif self.pos_y <= -0.01:
        #         self.pos_y -= 0.1726
        

    def slave_callback(self, msg : TwistStamped):
        # Lấy vận tốc hiện tại
        self.v = msg.twist.linear.x
        self.omega = msg.twist.angular.z

        # # Lấy thời gian hiện tại
        # time_now = self.get_clock().now()
        # dt = (time_now - self.time_prev).nanoseconds * 1e-9  # giây

        # if dt > 0.0:
        #     # Tính gia tốc
        #     self.a_v = (self.v - self.v_prev) / dt
        #     self.a_omega = (self.omega - self.omega_prev) / dt

        # # Cập nhật lại giá trị trước
        # self.v_prev = self.v
        # self.omega_prev = self.omega
        # self.time_prev = time_now

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

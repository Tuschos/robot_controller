#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import numpy as np

class FictitiousForceNode(Node):
    def __init__(self):
        super().__init__('fictitious_force_node')
        self.declare_parameter('robot_width', 0.23)   # m
        self.declare_parameter('delta', 0.2)         # m
        self.declare_parameter('force_gain', 12.0)
        self.declare_parameter('force_max', 0.30)

        self.c = self.get_parameter('robot_width').value
        self.delta = self.get_parameter('delta').value
        self.k = self.get_parameter('force_gain').value
        self.fv_max = self.get_parameter('force_max').value
        self.s_max = 0.7
        
        self.v = 0.0
        self.omega = 0.0
        self.fv_fil = 0.0
        self.alpha = 0.55    # He so loc thong thap

        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.slave_subscription = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        self.fictitious_force_pub = self.create_publisher(
            Float64,
            '/fictitious_force',
            10
        )

    def odom_callback(self, msg : Odometry):
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def lidar_max_with_angle(ranges, angles, segments=36):
        ranges = np.array(ranges)
        angles = np.array(angles)

        n = len(ranges)
        step = n // segments

        max_ranges = []
        max_angles = []

        for i in range(segments):
            start = i * step
            end = (i + 1) * step if i < segments - 1 else n

            idx = np.argmax(ranges[start:end])  # vị trí max trong đoạn

            max_ranges.append(ranges[idx + start])
            max_angles.append(angles[idx + start])

        return np.array(max_ranges), np.array(max_angles)

    def laser_callback(self, scan: LaserScan):
        v = self.v
        omega = self.omega
        c = self.c
        delta = self.delta
        k = self.k
        s_max = self.s_max

        # Tinh quy dao
        if abs(omega) < 0.001:  # coi nhu di thang
            r = 1e6
        else:
            r = abs(v / omega)

        # chuan hoa du lieu laser
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges)

        # Tinh toan luc ao
        fic_force = Float64()
        fv_raw = 0.0

        # Lay max range trong 36 segment
        ranges_max, angles_max = FictitiousForceNode.lidar_max_with_angle(ranges, angles)

        for i in range(len(ranges_max)):    
            l_i = ranges_max[i]
            theta_i = angles_max[i]
            
            #if v < 0:
                #if theta_i > 0:
                   # theta_i = theta_i - 3.14
                #else:
                  #  theta_i = theta_i + 3.14

            if theta_i > -1.57 and theta_i < 1.57:          
                # (Nếu robot đi thẳng, oi chính là điểm trên trục x)
                if abs(omega) < 0.001:
                    s_i = l_i * math.cos(theta_i)
                    d_i = abs(l_i * math.sin(theta_i))
                else:
                    # Quỹ đạo là cung tròn
                    # Tinh toan d_i va s_i
                    if omega < 0.0:
                        theta_i = - theta_i
                    center_to_li = math.sqrt(l_i*l_i + r*r - 2.0*l_i*r*math.cos(1.57 - theta_i))
                    d_i = abs(center_to_li - r)
                    phi_i = math.acos((center_to_li*center_to_li + r*r - l_i*l_i)/(2.0*center_to_li*r))
                    s_i = abs(r * phi_i)
                
                # He so trong so p_i
                if d_i <= c/2:
                    p_i = 1.0
                elif d_i < delta + c/2:
                    p_i = 0.5 * (1 + math.cos(math.pi * (d_i - c/2) / delta))
                else:
                    p_i = 0.0

                # Tong hop luc ao
                fv_raw += p_i * (s_max - s_i if s_i <= s_max else 0.0)
            
            else:
                continue
        
        # Luc ao raw chua loc
        fv_raw = (float) (k * fv_raw / len(ranges_max))

        #Loc thong thap lam muot luc ao
        self.fv_fil = self.fv_fil * self.alpha + (1 - self.alpha) * fv_raw

        #Log
        self.get_logger().info(f'v{self.v:.3f} : {self.omega:.3f} : {self.fv_fil}')
        # Gioi han tren do lon luc phan hoi ao
        if self.fv_fil > self.fv_max:
            self.fv_fil = self.fv_max

        # Neu v < 0 thi fv_raw am
        #if v < 0:
         #   fv_raw = -fv_raw
        
        fic_force.data = (float) (self.fv_fil)
        self.fictitious_force_pub.publish(fic_force)
        


def main(args=None):
    rclpy.init(args=args)
    node = FictitiousForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

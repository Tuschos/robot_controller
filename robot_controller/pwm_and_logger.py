#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import csv
import os


class PwmAndLogger(Node):
    def __init__(self):
        super().__init__('pwm_and_logger')

        # --------- Thông số robot (theo yêu cầu) ----------
        self.L = 0.175   # khoảng cách 2 bánh (m)
        self.r = 0.034   # bán kính bánh (m)

        # --------- Thông số PWM & thời gian ----------
        self.timer_period = 0.1       # publish /pwm mỗi 0.1s
        self.step_duration = 5.0      # mỗi mức PWM giữ 5 giây
        self.stabilize_time = 3.0     # 3 giây đầu: không log
        self.pwm_increment = 2.0      # tăng/giam 2% sau mỗi 5 giây
        self.min_pwm = -100.0
        self.max_pwm = 100.0

        self.pwm_value = 50.0          # bắt đầu từ 50
        self.step_start_time = None   # thời điểm bắt đầu mức PWM hiện tại
        self.logging_enabled = False  # chỉ bật trong 2s cuối của mỗi 5s

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --------- Publisher & Subscriber ----------
        self.pwm_pub = self.create_publisher(
            Float64MultiArray,
            '/pwm',
            10
        )

        self.vel_sub = self.create_subscription(
            TwistStamped,
            '/vel_encoder/data',
            self.vel_callback,
            qos_profile=qos
        )

        # --------- Biến lưu PWM hiện tại ----------
        self.last_pwm = 0.0

        # --------- Chuẩn bị file CSV ----------
        log_filename = '/home/tus/pwm_vel_log.csv'
        self.csv_file = open(log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Header file CSV
        self.csv_writer.writerow([
            'time_sec',
            'pwm',
            'omega_left',  # rad/s
            'omega_right'  # rad/s
        ])

        self.get_logger().info(f'Logging to {os.path.abspath(log_filename)}')
        self.get_logger().info(f'L = {self.L} m, r = {self.r} m')

        # --------- Timer để publish PWM + state machine thời gian ----------
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # ================== TIMER: publish PWM & điều khiển cửa sổ log ==================
    def timer_callback(self):
        now = self.get_clock().now()

        # Lần đầu gọi: khởi tạo step_start_time
        if self.step_start_time is None:
            self.step_start_time = now
            self.logging_enabled = False

        # Tính thời gian đã trôi qua trong step hiện tại
        elapsed = (now - self.step_start_time).nanoseconds * 1e-9  # giây

        # Điều khiển bật/tắt logging:
        #  - 0  →  3s: không log
        #  - 3 →  5s: bật log
        if elapsed < self.stabilize_time:
            self.logging_enabled = False
        elif elapsed < self.step_duration:
            self.logging_enabled = True
        else:
            # Hết 5 giây: chuyển sang mức PWM tiếp theo
            self.step_start_time = now
            self.logging_enabled = False

            # Tăng PWM thêm 2%, clamp trong [-100, 100]
            new_pwm = self.pwm_value + self.pwm_increment
            self.pwm_value = max(self.min_pwm, min(self.max_pwm, new_pwm))           #pwm duong
            # self.pwm_value = min(self.max_pwm, max(self.min_pwm, new_pwm))

            self.get_logger().info(
                f'New PWM step: {self.pwm_value:.1f}% (giữ 5s, log 2s cuối)'
            )

        # Publish PWM hiện tại ở mỗi tick
        msg = Float64MultiArray()
        msg.data = [self.pwm_value, self.pwm_value]  # 2 bánh như nhau
        self.pwm_pub.publish(msg)

        # Lưu lại để khi có /vel_encoder/data thì ghi log cùng
        self.last_pwm = self.pwm_value

    # ================== CALLBACK: nhận vận tốc encoder & ghi log ==================
    def vel_callback(self, msg: TwistStamped):
        # Chỉ ghi log trong khoảng logging_enabled (2 giây cuối của 5s)
        if not self.logging_enabled:
            return

        # Lấy v, omega của robot từ TwistStamped
        v = msg.twist.linear.x      # m/s
        omega = msg.twist.angular.z # rad/s

        # Tính vận tốc tuyến tính 2 bánh
        # v_r = v + omega * L/2
        # v_l = v - omega * L/2
        v_right = v + 0.5 * omega * self.L
        v_left = v - 0.5 * omega * self.L

        # Tính vận tốc góc 2 bánh
        omega_right = v_right / self.r
        omega_left = v_left / self.r

        # Lấy thời gian từ header
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Ghi 1 dòng vào CSV
        self.csv_writer.writerow([
            t,
            self.last_pwm,
            omega_left,
            omega_right
        ])

        # Flush để hạn chế mất dữ liệu khi node bị stop
        self.csv_file.flush()

    # ================== Dọn dẹp khi tắt node ==================
    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PwmAndLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

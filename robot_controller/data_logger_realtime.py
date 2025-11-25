#!/usr/bin/env python3
import math
import csv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64 
from nav_msgs.msg import Odometry
from control_msgs.msg import DynamicJointState

import matplotlib.pyplot as plt


class DataLoggerRealtimeNode(Node):
    def __init__(self):
        super().__init__('data_logger_realtime')

        # === Parameter: đường dẫn file CSV ===
        self.declare_parameter('csv_path', '/$HOME/teleop_log.csv')
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value

        # === Thời gian bắt đầu ===
        self.start_time = self.get_clock().now()

        # === Giá trị hiện tại (latest) từ các topic ===
        # Nếu chưa có dữ liệu thì để NaN
        self.master_pos = {'x': math.nan, 'y': math.nan, 'z': math.nan}
        self.robot_vel = {'v': math.nan, 'w': math.nan}
        self.master_force = {'x': math.nan, 'y': math.nan, 'z': math.nan}
        self.virtual_force = {'x': math.nan, 'y': math.nan, 'z': math.nan}

        # === Log dữ liệu theo thời gian ===
        self.t = []
        self.master_x = []
        self.master_y = []
        self.robot_v = []
        self.robot_w = []
        self.force_x = []
        self.force_y = []
        self.virtual_x = []


        # === Subscriber: CHỈNH LẠI TÊN TOPIC + MESSAGE TYPE CHO HỢP HỆ THỐNG CỦA BẠN ===
        # Vị trí tay cầm Novint Falcon (giả sử PoseStamped)
        self.create_subscription(DynamicJointState, '/fd/dynamic_joint_states_delayed',self.master_cb,10)

        # Vận tốc robot (giả sử Twist)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_cb, 10)

        # Lực phản hồi tại master (giả sử WrenchStamped)
        self.force_sub = self.create_subscription(Float64MultiArray, '/fd/fd_controller/commands', self.force_cb, 10)

        # Lực ảo (giả sử cũng WrenchStamped, bạn đổi theo hệ thống)
        self.fictitious_sub = self.create_subscription(Float64, '/fictitious_force', self.fictitious_cb, 10)

        # === Thiết lập matplotlib realtime ===
        plt.ion()
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.ax_master = self.axes[0, 0]
        self.ax_robot = self.axes[0, 1]
        self.ax_force = self.axes[1, 0]
        self.ax_virtual = self.axes[1, 1]

        # Master position
        self.line_master_x, = self.ax_master.plot([], [], label='x (m)')
        self.line_master_y, = self.ax_master.plot([], [], label='y (m)')
        self.ax_master.set_title('Vị trí tay cầm Novint Falcon')
        self.ax_master.set_ylabel('Position (m)')
        self.ax_master.legend()
        self.ax_master.grid(True)

        # Robot velocity
        self.line_robot_v, = self.ax_robot.plot([], [], label='v (m/s)')
        self.line_robot_w, = self.ax_robot.plot([], [], label='w (rad/s)')
        self.ax_robot.set_title('Vận tốc robot di động')
        self.ax_robot.set_ylabel('Velocity')
        self.ax_robot.legend()
        self.ax_robot.grid(True)

        # Master force feedback
        self.line_force_x, = self.ax_force.plot([], [], label='Fx (N)')
        self.line_force_y, = self.ax_force.plot([], [], label='Fy (N)')
        self.ax_force.set_title('Lực phản hồi tại master')
        self.ax_force.set_xlabel('Thời gian (s)')
        self.ax_force.set_ylabel('Force (N)')
        self.ax_force.legend()
        self.ax_force.grid(True)

        # Virtual force
        self.line_virtual_x, = self.ax_virtual.plot([], [], label='Fv')
        self.ax_virtual.set_title('Lực ảo (từ môi trường / lidar)')
        self.ax_virtual.set_xlabel('Thời gian (s)')
        self.ax_virtual.set_ylabel('Virtual force')
        self.ax_virtual.legend()
        self.ax_virtual.grid(True)

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # === Timer log + vẽ realtime (20 ms ~ 50 Hz) ===
        self.timer = self.create_timer(0.02, self.log_and_plot_cb)

        self.get_logger().info('TeleopDataLogger started. Logging and plotting...')

    # ===== Callback nhận dữ liệu =====
    def master_cb(self, msg: DynamicJointState):
        self.master_pos['x'] = msg.interface_values[0].values[0]
        self.master_pos['y'] = msg.interface_values[1].values[0]

    def odom_cb(self, msg: Odometry):
        # Giả sử dùng linear.x là vận tốc thẳng, angular.z là vận tốc quay
        self.robot_vel['v'] = msg.twist.twist.linear.x
        self.robot_vel['w'] = msg.twist.twist.angular.z

    def force_cb(self, msg: Float64MultiArray):
        self.master_force['x'] = msg.data[0]
        self.master_force['y'] = msg.data[1]

    def fictitious_cb(self, msg: Float64):
        self.virtual_force['x'] = msg.data


    # ===== Timer: log + vẽ realtime =====
    def log_and_plot_cb(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        # Ghi log
        self.t.append(t)
        self.master_x.append(self.master_pos['x'])
        self.master_y.append(self.master_pos['y'])

        self.robot_v.append(self.robot_vel['v'])
        self.robot_w.append(self.robot_vel['w'])

        self.force_x.append(self.master_force['x'])
        self.force_y.append(self.master_force['y'])

        self.virtual_x.append(self.virtual_force['x'])
        # Cập nhật line plot
        # 1. Master pos
        self.line_master_x.set_data(self.t, self.master_x)
        self.line_master_y.set_data(self.t, self.master_y)
        self.ax_master.relim()
        self.ax_master.autoscale_view()

        # 2. Robot vel
        self.line_robot_v.set_data(self.t, self.robot_v)
        self.line_robot_w.set_data(self.t, self.robot_w)
        self.ax_robot.relim()
        self.ax_robot.autoscale_view()

        # 3. Force feedback
        self.line_force_x.set_data(self.t, self.force_x)
        self.line_force_y.set_data(self.t, self.force_y)
        self.ax_force.relim()
        self.ax_force.autoscale_view()

        # 4. Virtual force
        self.line_virtual_x.set_data(self.t, self.virtual_x)
        self.ax_virtual.relim()
        self.ax_virtual.autoscale_view()

        # Vẽ
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        # Cho matplotlib xử lý event GUI
        plt.pause(0.001)

    # ===== Lưu CSV khi thoát =====
    def save_to_csv(self):
        n_samples = len(self.t)
        self.get_logger().info(f'Saving {n_samples} samples to {self.csv_path}')
        try:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'time_s',
                    'master_x', 'master_y',
                    'robot_v', 'robot_w',
                    'force_x', 'force_y',
                    'virtual_x'
                ])
                for i in range(n_samples):
                    writer.writerow([
                        self.t[i],
                        self.master_x[i], self.master_y[i],
                        self.robot_v[i], self.robot_w[i],
                        self.force_x[i], self.force_y[i],
                        self.virtual_x[i],
                    ])
            self.get_logger().info('CSV saved successfully.')
        except Exception as e:
            self.get_logger().error(f'Error saving CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerRealtimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_to_csv()
        node.destroy_node()
        rclpy.shutdown()
        # Để cửa sổ plot không tắt ngay
        print("Node stopped. Close the plot window to exit.")
        plt.ioff()
        plt.show()


if __name__ == '__main__':
    main()

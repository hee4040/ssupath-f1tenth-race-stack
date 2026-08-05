import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from scipy.signal import butter, lfilter, lfilter_zi
import csv
import os

class CollectDataDDM(Node):
    def __init__(self):
        super().__init__('collect_data_ddm')


        self.declare_parameter('save_dir','/home/misys/forza_ws/race_stack/system_identification/deep_dynamics/data')
        self.declare_parameter('csv_name','teras.csv')

        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.csv_name = self.get_parameter('csv_name').get_parameter_value().string_value
        os.makedirs(self.save_dir, exist_ok=True)


        self.get_logger().info(f"CSV save dir : {self.save_dir}")
        self.get_logger().info(f"CSV file name: {self.csv_name}")

        self.create_subscription(Odometry, '/car_state/odom', self.odom_callback, 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self.steer_callback, 10)
        self.create_subscription(VescStateStamped, '/sensors/core', self.duty_cycle_callback, 10)

        self.prev_steer = 0.
        self.steer = 0.
        self.delta_steer = 0.
        self.duty_cycle = 0.

        self.prev_time = 0.
        self.prev_x = 0.
        self.prev_y = 0.
        self.prev_yaw = 0.
        self.prev_vx = 0.

        self.data = []

        

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def steer_callback(self, msg):
        self.steer = msg.drive.steering_angle

    def duty_cycle_callback(self, msg):
        self.duty_cycle = msg.state.duty_cycle

    def odom_callback(self, msg):
        # time
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # pose
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, curr_yaw = euler_from_quaternion(quat)

        # twist
        curr_vx = msg.twist.twist.linear.x
        curr_vy = msg.twist.twist.linear.y
        curr_omega = msg.twist.twist.angular.z

        # inputs
        steering = self.steer
        throttle = self.duty_cycle
        brake = 0.0

        # optional features (미사용이면 0.0)
        ax = 0.0
        deltadelta = 0.0

        # 헤더와 정확히 동일한 순서/개수로 기록
        self.data.append([
            curr_time,          # time
            curr_x,             # x
            curr_y,             # y
            curr_vx,            # vx
            curr_vy,            # vy
            curr_yaw,           # phi
            steering,           # delta
            curr_omega,         # omega
            ax,                 # ax
            deltadelta,         # deltadelta
            roll,               # roll
            throttle,           # throttle_ped_cmd (여기선 duty_cycle)
            brake               # brake_ped_cmd
        ])

    def save_to_csv(self):
        if not self.data:
            self.get_logger().warn("No data to save.")
            return

        filepath = os.path.join(self.save_dir, self.csv_name)

        with open(filepath, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "time", "x", "y", "vx", "vy", "phi",
                "delta", "omega", "ax", "deltadelta",
                "roll", "throttle_ped_cmd", "brake_ped_cmd"
            ])
            for row in self.data:
                writer.writerow(row)

        self.get_logger().info(f"Saved data to {filepath}")

def main(args=None):
    rclpy.init(args=args)
    node = CollectDataDDM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_to_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
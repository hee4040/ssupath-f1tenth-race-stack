#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import yaml
import csv
import numpy as np
from datetime import datetime
from tqdm import tqdm

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

# from helpers.train_model import nn_train

from on_track_sys_id.train_model import nn_train
from ament_index_python.packages import get_package_share_directory



class OnTrackSysId(Node):
    def __init__(self):
        super().__init__('on_track_sys_id')

        # Declare parameters with default values
        self.declare_parameter('racecar_version', 'SIM')
        self.declare_parameter('save_LUT_name', 'NUCx_on_track_pacejka')
        self.declare_parameter('save_dir', '/home/misys/forza_ws/race_stack/controller/map/lut')
        self.declare_parameter('plot_model', True)
        self.declare_parameter('odom_topic', '/car_state/odom')
        self.declare_parameter('ackermann_cmd_topic', '/vesc/high_level/ackermann_cmd_mux/input/nav_1')

        # Get parameters
        self.racecar_version = self.get_parameter('racecar_version').value
        self.save_LUT_name = self.get_parameter('save_LUT_name').value
        self.save_dir = self.get_parameter('save_dir').value
        self.plot_model = self.get_parameter('plot_model').value

        self.get_logger().info(f"Racecar version: {self.racecar_version}")

        self.rate = 50.0  # Hz
        # self.package_path = os.path.dirname(os.path.abspath(__file__))
        self.package_path = get_package_share_directory('on_track_sys_id')

        self.load_parameters()
        self.setup_data_storage()

        # Subscribe to topics
        odom_topic = self.get_parameter('odom_topic').value
        ackermann_topic = self.get_parameter('ackermann_cmd_topic').value

        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(AckermannDriveStamped, ackermann_topic, self.ackermann_cb, 10)

        # Timer for loop
        self.timer = self.create_timer(1.0 / self.rate, self.collect_data)

        # Progress bar for data collection
        self.pbar = tqdm(total=self.timesteps, desc='Collecting data', ascii=True)

    def setup_data_storage(self):
        """ Initialize data storage based on parameters. """
        self.data_duration = self.nn_params['data_collection_duration']
        self.timesteps = int(self.data_duration * self.rate)
        self.data = np.zeros((self.timesteps, 4))
        self.counter = 0
        self.current_state = np.zeros(4)

    def load_parameters(self):
        """ Load neural network parameters from YAML file. """
        yaml_file = os.path.join(self.package_path, 'params/nn_params.yaml')
        with open(yaml_file, 'r') as file:
            self.nn_params = yaml.safe_load(file)

    def export_data_as_csv(self):
        """ Export collected data to CSV file. """
        user_input = input("\033[33m[WARN] Press 'Y' and ENTER to export data as CSV, or just ENTER to continue: \033[0m")
        if user_input.lower() == 'y':
            data_dir = os.path.join(self.package_path, 'data', self.racecar_version)
            os.makedirs(data_dir, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_file = os.path.join(data_dir, f'{self.racecar_version}_sys_id_data_{timestamp}.csv')

            with open(csv_file, mode='w') as file:
                writer = csv.writer(file)
                writer.writerow(['v_x', 'v_y', 'omega', 'delta'])
                writer.writerows(self.data)
            
            self.get_logger().info(f"DATA HAS BEEN EXPORTED TO: {csv_file}")

    def odom_cb(self, msg):
        """ Odometry callback function. """
        self.current_state[0] = msg.twist.twist.linear.x
        self.current_state[1] = msg.twist.twist.linear.y
        self.current_state[2] = msg.twist.twist.angular.z

    def ackermann_cb(self, msg):
        """ Ackermann command callback function. """
        self.current_state[3] = msg.drive.steering_angle

    def collect_data(self):
        """ Collects data during simulation and trains the model when collection is complete. """
        if self.current_state[0] > 1:  # Only collect data when the car is moving
            self.data = np.roll(self.data, -1, axis=0)
            self.data[-1] = self.current_state
            self.counter += 1
            self.pbar.update(1)

        if self.counter >= self.timesteps:
            self.pbar.close()
            self.get_logger().info("Data collection completed.")
            self.run_nn_train()
            self.export_data_as_csv()
            self.get_logger().info("Training completed. Shutting down...")
            self.destroy_node()
            rclpy.shutdown()

    def run_nn_train(self):
        """ Train the neural network using collected data. """
        self.get_logger().info("Training neural network...")
        nn_train(self.data, self.racecar_version, self.save_LUT_name, self.save_dir, self.plot_model)


def main(args=None):
    rclpy.init(args=args)
    sys_id = OnTrackSysId()
    rclpy.spin(sys_id)
    sys_id.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

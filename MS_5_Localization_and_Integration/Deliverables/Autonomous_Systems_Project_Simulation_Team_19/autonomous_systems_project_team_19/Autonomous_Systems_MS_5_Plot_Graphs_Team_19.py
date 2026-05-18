#!/usr/bin/env python3
import csv
import math
import os

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node


class MS5PlotGraphs(Node):
    def __init__(self):
        super().__init__('Autonomous_Systems_MS_5_Plot_Graphs_Team_19')
        self.declare_parameter('track_id', 2)
        self.track_id = int(self.get_parameter('track_id').value)
        self.folder = os.path.expanduser(f'~/ros2_ws/graphs_track{self.track_id}')
        self.csv_path = os.path.join(self.folder, f'Autonomous_Systems_Project_Simulation_MS_5_Track_{self.track_id}_Log_Team_19.csv')
        self.plot_all()

    def read_csv(self):
        if not os.path.exists(self.csv_path):
            raise FileNotFoundError(f'CSV log not found: {self.csv_path}')
        with open(self.csv_path, 'r') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        data = {name: [] for name in reader.fieldnames}
        for row in rows:
            for name, value in row.items():
                try:
                    data[name].append(float(value))
                except Exception:
                    data[name].append(math.nan)
        return data

    def save_plot(self, data, y_keys, title, ylabel, filename):
        plt.figure(figsize=(10, 6))
        for key, label in y_keys:
            plt.plot(data['time'], data[key], label=label)
        plt.title(title)
        plt.xlabel('Time [s]')
        plt.ylabel(ylabel)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        path = os.path.join(self.folder, filename)
        plt.savefig(path, dpi=200)
        plt.close()
        self.get_logger().info(f'Saved: {path}')

    def plot_all(self):
        data = self.read_csv()
        os.makedirs(self.folder, exist_ok=True)
        prefix = f'MS5_track{self.track_id}'
        self.save_plot(
            data,
            [('ideal_x', 'Ideal x'), ('noisy_x', 'Noisy x'), ('filtered_x', 'Filtered x')],
            f'MS5 Track {self.track_id}: x State Before/After Filter',
            'x [m]',
            f'{prefix}_x_ideal_noisy_filtered_Team_19.png',
        )
        self.save_plot(
            data,
            [('desired_y', 'Desired lane y'), ('ideal_y', 'Ideal y'), ('noisy_y', 'Noisy y'), ('filtered_y', 'Filtered y')],
            f'MS5 Track {self.track_id}: Lane Tracking',
            'y [m]',
            f'{prefix}_y_lane_tracking_Team_19.png',
        )
        self.save_plot(
            data,
            [('ideal_theta', 'Ideal theta'), ('noisy_theta', 'Noisy theta'), ('filtered_theta', 'Filtered theta')],
            f'MS5 Track {self.track_id}: Heading Before/After Filter',
            'theta [rad]',
            f'{prefix}_theta_ideal_noisy_filtered_Team_19.png',
        )
        self.save_plot(
            data,
            [('desired_speed', 'Desired speed'), ('ideal_speed', 'Ideal speed'), ('filtered_speed', 'Filtered speed'), ('speed_cmd', 'Speed command')],
            f'MS5 Track {self.track_id}: Speed Response',
            'speed [m/s]',
            f'{prefix}_speed_response_Team_19.png',
        )
        self.save_plot(
            data,
            [('steering_cmd', 'Steering command')],
            f'MS5 Track {self.track_id}: Steering Command',
            'steering [rad]',
            f'{prefix}_steering_command_Team_19.png',
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MS5PlotGraphs()
    except Exception as e:
        print(f'ERROR while plotting MS5 graphs: {e}')
        rclpy.shutdown()
        return
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

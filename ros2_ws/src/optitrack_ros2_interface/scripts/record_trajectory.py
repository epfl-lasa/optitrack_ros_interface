#!/usr/bin/env python3

import csv
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class OptitrackPoseRecorder(Node):

    def __init__(self):
        super().__init__('optitrack_pose_recorder')

        self.declare_parameter('topic', '/ball_19/pose')
        self.declare_parameter(
            'output_directory',
            '/home/ros2/ros2_ws/records'
        )
        self.declare_parameter('file_name', '')

        topic = self.get_parameter('topic').value
        output_directory = self.get_parameter('output_directory').value
        file_name = self.get_parameter('file_name').value

        os.makedirs(output_directory, exist_ok=True)

        if not file_name:
            current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
            file_name = f'optitrack_pose_{current_time}.csv'

        self.output_path = os.path.join(output_directory, file_name)

        self.csv_file = open(
            self.output_path,
            mode='w',
            newline='',
            buffering=1
        )

        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            'timestamp_sec',
            'elapsed_time_sec',
            'frame_id',
            'position_x',
            'position_y',
            'position_z',
            'quaternion_x',
            'quaternion_y',
            'quaternion_z',
            'quaternion_w',
        ])

        self.first_timestamp = None
        self.sample_count = 0

        # VRPN pose topics commonly use best-effort QoS.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            topic,
            self.pose_callback,
            qos
        )

        self.get_logger().info(f'Recording topic: {topic}')
        self.get_logger().info(f'Saving CSV to: {self.output_path}')

    def pose_callback(self, msg: PoseStamped):
        timestamp_sec = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )

        if self.first_timestamp is None:
            self.first_timestamp = timestamp_sec

        elapsed_time_sec = timestamp_sec - self.first_timestamp

        self.writer.writerow([
            f'{timestamp_sec:.9f}',
            f'{elapsed_time_sec:.9f}',
            msg.header.frame_id,
            f'{msg.pose.position.x:.10f}',
            f'{msg.pose.position.y:.10f}',
            f'{msg.pose.position.z:.10f}',
            f'{msg.pose.orientation.x:.10f}',
            f'{msg.pose.orientation.y:.10f}',
            f'{msg.pose.orientation.z:.10f}',
            f'{msg.pose.orientation.w:.10f}',
        ])

        self.sample_count += 1

        if self.sample_count % 100 == 0:
            self.get_logger().info(
                f'Recorded {self.sample_count} samples'
            )

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(
                f'CSV closed: {self.output_path}'
            )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = OptitrackPoseRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Recording stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
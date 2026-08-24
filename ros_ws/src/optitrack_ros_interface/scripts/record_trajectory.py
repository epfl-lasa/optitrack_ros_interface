#!/usr/bin/env python3
"""Record one OptiTrack ``PoseStamped`` topic to a CSV file.

Run with 'rosrun optitrack_ros_interface record_trajectory.py' and override
the private parameters ``~topic``, ``~output_directory``, and ``~file_name`` as
needed. Recording continues until the node is shut down (with Ctrl+C).

For example: 
    rosrun optitrack_ros_interface record_trajectory.py 
    _topic:=/vrpn_client_node/holy_cup/pose \
    _output_directory:=/home/ros/ros_ws/data\
    _file_name:=holy_cup.csv
"""

import csv
import os
import threading
from datetime import datetime

import rospy
from geometry_msgs.msg import PoseStamped


class OptitrackPoseRecorder:
    """Subscribe to one pose topic and write every received pose to CSV."""

    def __init__(self):
        """Read ROS parameters, open the output file, and start subscribing."""
        topic = rospy.get_param('~topic', '/vrpn_client_node/holy_cup/pose')
        output_directory = rospy.get_param(
            '~output_directory', '/home/ros/ros_ws/data'
        )
        file_name = rospy.get_param('~file_name', '')

        os.makedirs(output_directory, exist_ok=True)

        if not file_name:
            current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
            topic_name = topic.strip('/').replace('/', '_') or 'topic'
            file_name = f'{topic_name}_{current_time}.csv'
        else:
            # Keep explicitly named recordings inside output_directory.
            file_name = os.path.basename(file_name)
            if file_name in ('', '.', '..'):
                raise ValueError('file_name must name a file')

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
        self.file_lock = threading.Lock()

        self.subscription = rospy.Subscriber(
            topic,
            PoseStamped,
            self.pose_callback,
            queue_size=10
        )
        rospy.on_shutdown(self.destroy_node)

        rospy.loginfo(f'Recording topic: {topic}')
        rospy.loginfo(f'Saving CSV to: {self.output_path}')

    def pose_callback(self, msg: PoseStamped):
        """Append a received pose and its timestamp to the output CSV."""
        with self.file_lock:
            if self.csv_file.closed:
                return

            timestamp_sec = msg.header.stamp.to_sec()

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
                rospy.loginfo(
                    f'Recorded {self.sample_count} samples'
                )

    def destroy_node(self):
        """Stop the subscription and safely flush and close the CSV file."""
        if hasattr(self, 'subscription'):
            self.subscription.unregister()

        with self.file_lock:
            if hasattr(self, 'csv_file') and not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()

                if not rospy.is_shutdown():
                    rospy.loginfo(
                        f'CSV closed: {self.output_path}'
                    )
                else:
                    print(f'CSV closed: {self.output_path}')


def main():
    """Initialize the ROS node and record poses until shutdown."""
    rospy.init_node('optitrack_pose_recorder')

    node = OptitrackPoseRecorder()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        if not rospy.is_shutdown():
            rospy.loginfo('Recording stopped')
        else:
            print('Recording stopped')
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

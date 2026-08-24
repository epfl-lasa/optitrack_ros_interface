#!/usr/bin/env python3
"""Record synchronized OptiTrack ``PoseStamped`` topics to one CSV file.

Run with 'rosrun optitrack_ros_interface record_multiple_trajectory.py'.
The private ``~topics`` parameter must contain at least two unique topic names.
``~queue_size`` and ``~sync_slop_sec`` control approximate synchronization;
``~output_directory`` and ``~file_name`` select the output file. Recording
continues until the node is shut down (for example, with Ctrl-C).

For example:
    rosrun optitrack_ros_interface record_multiple_trajectory.py \
    _topics:="[/vrpn_client_node/object_1/pose, /vrpn_client_node/object_2/pose]" \
    _output_directory:=/home/ros/ros_ws/data \
    _file_name:=object1_and_object2.csv
"""

import csv
import os
import threading
from datetime import datetime

import message_filters
import rospy
from geometry_msgs.msg import PoseStamped


class OptitrackMultiplePoseRecorder:
    """Synchronize poses from several objects and write each group to CSV."""

    def __init__(self):
        """Read ROS parameters, configure synchronization, and open the CSV."""
        self.topics = rospy.get_param(
            '~topics',
            [
                '/vrpn_client_node/object_1/pose',
                '/vrpn_client_node/object_2/pose',
            ],
        )
        output_directory = rospy.get_param(
            '~output_directory', '/home/ros/ros_ws/data'
        )
        file_name = rospy.get_param('~file_name', '')
        queue_size = rospy.get_param('~queue_size', 30)
        sync_slop_sec = rospy.get_param('~sync_slop_sec', 0.01)

        if not isinstance(self.topics, list) or len(self.topics) < 2:
            raise ValueError('~topics must be a list containing at least two topics')
        if len(set(self.topics)) != len(self.topics):
            raise ValueError('~topics must not contain duplicate topics')

        self.object_names = [self._object_name(topic) for topic in self.topics]
        if len(set(self.object_names)) != len(self.object_names):
            raise ValueError(
                'Topic-derived object names must be unique: {}'.format(
                    self.object_names
                )
            )

        os.makedirs(output_directory, exist_ok=True)

        if not file_name:
            current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
            file_name = 'multiple_objects_{}.csv'.format(current_time)
        else:
            file_name = os.path.basename(file_name)
            if file_name in ('', '.', '..'):
                raise ValueError('file_name must name a file')

        self.output_path = os.path.join(output_directory, file_name)
        self.csv_file = open(
            self.output_path,
            mode='w',
            newline='',
            buffering=1,
        )
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(self._csv_header())

        self.first_timestamp = None
        self.sample_count = 0
        self.file_lock = threading.Lock()

        self.subscriptions = [
            message_filters.Subscriber(topic, PoseStamped)
            for topic in self.topics
        ]
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.subscriptions,
            queue_size=queue_size,
            slop=sync_slop_sec,
        )
        self.synchronizer.registerCallback(self.pose_callback)
        rospy.on_shutdown(self.close)

        rospy.loginfo('Recording synchronized topics: %s', self.topics)
        rospy.loginfo('Synchronization tolerance: %.6f seconds', sync_slop_sec)
        rospy.loginfo('Saving CSV to: %s', self.output_path)

    @staticmethod
    def _object_name(topic):
        """Derive a stable CSV column prefix from a pose topic name."""
        parts = [part for part in topic.split('/') if part]
        if not parts:
            raise ValueError('Topic names must not be empty')
        return parts[-2] if len(parts) >= 2 and parts[-1] == 'pose' else parts[-1]

    def _csv_header(self):
        """Build CSV columns for timing and each configured tracked object."""
        header = ['timestamp_sec', 'elapsed_time_sec', 'frame_id']
        for name in self.object_names:
            header.extend([
                '{}.position_x'.format(name),
                '{}.position_y'.format(name),
                '{}.position_z'.format(name),
                '{}.quaternion_x'.format(name),
                '{}.quaternion_y'.format(name),
                '{}.quaternion_z'.format(name),
                '{}.quaternion_w'.format(name),
            ])
        return header

    def pose_callback(self, *messages):
        """Append one approximately synchronized group of poses to the CSV."""
        with self.file_lock:
            if self.csv_file.closed:
                return

            # Use the latest stamp as the timestamp for this synchronized group.
            timestamp_sec = max(msg.header.stamp.to_sec() for msg in messages)
            if self.first_timestamp is None:
                self.first_timestamp = timestamp_sec

            row = [
                '{:.9f}'.format(timestamp_sec),
                '{:.9f}'.format(timestamp_sec - self.first_timestamp),
                messages[0].header.frame_id,
            ]
            for msg in messages:
                row.extend([
                    '{:.10f}'.format(msg.pose.position.x),
                    '{:.10f}'.format(msg.pose.position.y),
                    '{:.10f}'.format(msg.pose.position.z),
                    '{:.10f}'.format(msg.pose.orientation.x),
                    '{:.10f}'.format(msg.pose.orientation.y),
                    '{:.10f}'.format(msg.pose.orientation.z),
                    '{:.10f}'.format(msg.pose.orientation.w),
                ])

            self.writer.writerow(row)
            self.sample_count += 1
            if self.sample_count % 100 == 0:
                rospy.loginfo('Recorded %d synchronized samples', self.sample_count)

    def close(self):
        """Flush and close the output CSV during ROS shutdown."""
        with self.file_lock:
            if hasattr(self, 'csv_file') and not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()
                rospy.loginfo(
                    'CSV closed after %d synchronized samples: %s',
                    self.sample_count,
                    self.output_path,
                )


def main():
    """Initialize the ROS node and record synchronized poses until shutdown."""
    rospy.init_node('optitrack_multiple_pose_recorder')
    OptitrackMultiplePoseRecorder()
    rospy.spin()


if __name__ == '__main__':
    main()

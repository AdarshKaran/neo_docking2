#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformListener
import tf2_ros
import tf2_geometry_msgs
from rclpy.action import ActionClient
from opennav_docking_msgs.action import DockRobot
import os
import yaml
from ament_index_python.packages import get_package_share_directory


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initially, don't publish until triggered by feedback
        self.publish_pose = False
        # Subscription to trigger publishing
        self.start_publishing_subscription = self.create_subscription(
            Bool,
            'start_publishing_dock_pose',
            self.start_publishing_callback,
            10
        )
        self.get_logger().info('PosePublisher node has been started.')
        # Load the dock coordinates from the YAML file
        self.load_dock_pose()

    def load_dock_pose(self):
        """Load the dock pose from the YAML file."""
        try:
            package_share_directory = get_package_share_directory('neo_docking2')
            yaml_file = os.path.join(package_share_directory, "config", "dock_database.yaml")
            with open(yaml_file, 'r') as file:
                yaml_content = yaml.safe_load(file)
                dock_id = 'dock1'
                if 'docks' in yaml_content and dock_id in yaml_content['docks']:
                    dock_data = yaml_content['docks'][dock_id]
                    pose_list = dock_data['pose']
                    self.dock_pose_x = pose_list[0]
                    self.dock_pose_y = pose_list[1]
                    self.dock_pose_z = 0.0
                    self.get_logger().info(f"Loaded dock pose: x={self.dock_pose_x}, y={self.dock_pose_y}, z={self.dock_pose_z}")
                else:
                    self.get_logger().error(f"Dock ID '{dock_id}' not found in the YAML file.")
                    self.dock_pose_x = 0.0
                    self.dock_pose_y = 0.0
                    self.dock_pose_z = 0.0
        except Exception as e:
            self.get_logger().error(f"Error loading dock pose from YAML file: {e}")
            self.dock_pose_x = 0.0
            self.dock_pose_y = 0.0
            self.dock_pose_z = 0.0

    def start_publishing_callback(self, msg):
        """Callback when receiving trigger to start publishing."""
        if msg.data:
            # self.get_logger().info('Received start signal, will begin publishing poses.')
            self.publish_pose = True
        else:
            self.get_logger().info('Received stop signal, will stop publishing poses.')
            self.publish_pose = False

    def timer_callback(self):
        """Publishes the transformed pose if the INITIAL_PERCEPTION state has been reached."""
        if self.publish_pose:
            # Define the pose in the map frame
            pose_in_map = tf2_geometry_msgs.PoseStamped()
            pose_in_map.header = Header()
            pose_in_map.header.stamp = self.get_clock().now().to_msg()
            pose_in_map.header.frame_id = 'map'
            pose_in_map.pose.position.x = self.dock_pose_x
            pose_in_map.pose.position.y = self.dock_pose_y
            pose_in_map.pose.position.z = self.dock_pose_z

            try:
                # Check if transformation between 'odom' and 'map' is available
                if not self.tf_buffer.can_transform('odom', 'map', rclpy.time.Time()):
                    self.get_logger().error('No transform from odom to map yet.')
                    return

                # Perform the transformation
                transform_stamped = self.tf_buffer.lookup_transform(
                    'odom', 'map', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)
                )
                pose_in_odom = tf2_geometry_msgs.do_transform_pose(pose_in_map.pose, transform_stamped)

                # Publish the transformed pose
                final_pose_stamped = tf2_geometry_msgs.PoseStamped()
                final_pose_stamped.header = Header()
                final_pose_stamped.header.stamp = transform_stamped.header.stamp
                final_pose_stamped.header.frame_id = 'odom'
                final_pose_stamped.pose = pose_in_odom
                final_pose_stamped.pose.orientation.w = 0.0
                final_pose_stamped.pose.orientation.z = 1.0
                self.publisher_.publish(final_pose_stamped)
                # self.get_logger().info(f'Published PoseStamped in odom frame: {final_pose_stamped.pose}')

            except Exception as e:
                self.get_logger().error(f'Error transforming pose: {e}')

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

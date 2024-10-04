#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, Header
from tf2_ros import Buffer, TransformListener
import tf2_ros
import tf2_geometry_msgs
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import tf_transformations
import math


class PosePublisher(Node):
    """Node to publish the dock pose as feedback while docking."""
    def __init__(self):
        super().__init__('pose_publisher')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.detected_dock_pose_publisher = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)
        # Subscription to trigger publishing
        self.start_publishing_subscription = self.create_subscription(
            Bool,
            '/start_publishing_dock_pose',
            self.start_publishing_callback,
            10
        )
        # Subscriber for target_dock_id
        self.dock_id_subscriber = self.create_subscription(
            String,
            '/target_dock_id',
            self.dock_id_callback,
            10
        )

        # Don't publish dock pose until triggered by feedback
        self.publish_pose = False
        self.current_dock_id = None
        # Dictionary to store dock poses to avoid repeated file reads
        self.dock_poses = {}
        # Load all dock poses from the YAML file
        self.load_all_dock_poses()
        self.get_logger().info('PosePublisher node has been started.')

    def dock_id_callback(self, msg):
        """Callback to receive the current dock_id."""
        new_dock_id = msg.data
        if new_dock_id != self.current_dock_id:
            self.current_dock_id = new_dock_id
            self.get_logger().info(f"Received new dock_id: {self.current_dock_id}")

    def load_all_dock_poses(self):
        """Load all dock poses from the YAML file into a dictionary."""
        try:
            package_share_directory = get_package_share_directory('neo_docking2')
            yaml_file = os.path.join(package_share_directory, "config", "dock_database.yaml")
            with open(yaml_file, 'r') as file:
                yaml_content = yaml.safe_load(file)
                if 'docks' in yaml_content:
                    for dock_id, dock_data in yaml_content['docks'].items():
                        pose_list = dock_data['pose']  # This is [x, y, theta]
                        pose = {
                            'x': pose_list[0],
                            'y': pose_list[1],
                            'theta': pose_list[2]
                        }
                        self.dock_poses[dock_id] = pose
                        self.get_logger().info(f"Loaded dock '{dock_id}': position=({pose['x']}, "
                                            f"{pose['y']}), theta={pose['theta']}")
                else:
                    self.get_logger().error("No 'docks' key found in the YAML file.")
        except Exception as e:
            self.get_logger().error(f"Error loading dock poses from YAML file: {e}")
            self.set_default_pose()

    def set_default_pose(self):
        """Set default pose value."""
        self.dock_poses = {
            'dock1': {
                'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            }
        }
        self.get_logger().info("Default dock pose set.")

    def start_publishing_callback(self, msg):
        """Callback when receiving trigger to start publishing."""
        if msg.data:
            # self.get_logger().info('Received start signal, will begin publishing poses.')
            self.publish_pose = True
        else:
            self.get_logger().info('Received stop signal, will stop publishing poses.')
            self.publish_pose = False

    def timer_callback(self):
        """Publishes the transformed pose."""
        if self.publish_pose and self.current_dock_id:
            # Retrieve the pose for the current dock_id
            dock_pose_data = self.dock_poses.get(self.current_dock_id, None)
            if dock_pose_data is None:
                self.get_logger().error(f"No pose data found for dock_id '{self.current_dock_id}'.")
                return
            
            # Create PoseStamped message
            pose_in_map = PoseStamped()
            pose_in_map.header = Header()
            pose_in_map.header.stamp = self.get_clock().now().to_msg()
            pose_in_map.header.frame_id = 'map'
            pose_in_map.pose.position.x = dock_pose_data['x']
            pose_in_map.pose.position.y = dock_pose_data['y']
            pose_in_map.pose.position.z = 0.0  # Assuming z is 0 since 2D

            theta = dock_pose_data['theta']
            # Offset of 180 degrees to account for robot docking backwards
            theta = theta + math.pi
            # Normalize theta to range [-π, π]
            theta = (theta + math.pi) % (2 * math.pi) - math.pi

            # Calculate quaternion from adjusted theta
            quaternion = tf_transformations.quaternion_from_euler(0, 0, theta)

            # Assign the orientation
            pose_in_map.pose.orientation.x = quaternion[0]
            pose_in_map.pose.orientation.y = quaternion[1]
            pose_in_map.pose.orientation.z = quaternion[2]
            pose_in_map.pose.orientation.w = quaternion[3]

            self.get_logger().info(f"Pose data for dock_id '{self.current_dock_id}': {pose_in_map}")

            try:
                # Check if transformation between 'odom' and 'map' is available
                if not self.tf_buffer.can_transform('odom', 'map', rclpy.time.Time()):
                    self.get_logger().error('No transform from odom to map available yet.')
                    return

                # Perform the transformation
                transform_stamped = self.tf_buffer.lookup_transform(
                    'odom', 'map', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)
                )
                pose_in_odom = tf2_geometry_msgs.do_transform_pose(pose_in_map.pose, transform_stamped)

                # Publish the transformed pose
                final_pose_stamped = PoseStamped()
                final_pose_stamped.header = Header()
                final_pose_stamped.header.stamp = transform_stamped.header.stamp
                final_pose_stamped.header.frame_id = 'odom'
                final_pose_stamped.pose = pose_in_odom

                # Publish the transformed pose
                self.detected_dock_pose_publisher.publish(final_pose_stamped)
                self.get_logger().debug(f"Published pose for dock_id '{self.current_dock_id}': {final_pose_stamped.pose}")

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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException, TransformException
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from neo_srvs2.srv import SaveDockPose

# Custom list class for inline lists
class InlineList(list):
    pass

# Custom representer to dump InlineList in flow style
def represent_inline_list(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

# Register the custom representer
yaml.add_representer(InlineList, represent_inline_list)

class SaveDockPoseToYaml(Node):
    def __init__(self):
        super().__init__('save_dock_pose_to_yaml_node')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_service(SaveDockPose, "save_dock_pose_to_yaml", self.save_pose)
        self.dock_poses = {}
        # Define the output file path
        package_share_directory = get_package_share_directory('neo_docking2')
        self.output_file = os.path.join(package_share_directory, "config", "dock_database.yaml")

    def save_pose(self, request, response):
        dock_id = request.dock_id
        dock_type = request.dock_type
        dock_frame = request.dock_frame

        when = rclpy.time.Time()

        try:
            if not self.tf_buffer.can_transform('map', 'base_link', when):
                self.get_logger().error('No transform from map to base_link yet.')
                response.success = False
                response.message = "No transform from map to base_link yet"
                return response

            transform_stamped = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
            # Use InlineList for the pose to enforce flow style
            pose_list = InlineList([
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z
            ])
            # Store the pose data in a dictionary with the desired format
            pose_data = {
                'type': dock_type,
                'frame': dock_frame,
                'pose': pose_list
            }
            
            # Add or update the dock pose
            self.dock_poses[dock_id] = pose_data

            # Write the updated dock poses to the YAML file
            response.success = self.write_to_yaml()
            if not response.success:
                response.message = "Error writing to YAML file."
                return response
            response.message = "Dock pose saved to YAML file successfully."
            return response

        except (LookupException, ConnectivityException, ExtrapolationException, TransformException) as e:
            response.success = False
            response.message = str(e)
            return response

    def write_to_yaml(self):
        try:
            # Write the dock poses to the YAML file in the desired format
            with open(self.output_file, 'w') as file:
                yaml.dump({'docks': self.dock_poses}, file, default_flow_style=False)
            self.get_logger().info("Dock pose saved to YAML file.")
            return True
        except Exception as e:
            self.get_logger().error(f"Error writing to YAML file: {str(e)}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = SaveDockPoseToYaml()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

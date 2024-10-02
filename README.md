# Neo Docking with OpenNav Docking Server

This repository contains the necessary files and configurations to simulate and run docking on the MPO 700 robot in Gazebo using OpenNav's docking server.

## Installation

### Prerequisites

Before proceeding, ensure that you have cloned the following repositories into your ROS2 workspace:

   1. Clone the neo_docking2 repository:<br>
      `git clone -b update/neo_docking_with_opennav_docking_server https://github.com/AdarshKaran/neo_docking2.git`
   3. Clone the opennav_docking repository:<br>
      `git clone -b humble_fix/neo_dock https://github.com/AdarshKaran/opennav_docking.git`<br>
      
If you haven't installed the Neobotix packages, please follow the installation guide here: [https://neobotix-docs.de/ros/ros2/installation.html]<br>

Once you have the neo_simulation2, neo_nav2_bringup, neo_docking2, and opennav_docking packages installed, proceed to the next steps.

## Running the Simulation

1. **Launch the Gazebo Simulation:**

   This command launches the Gazebo Classic simulation with the MPO 700 robot in the `neo_workshop` world:

   `ros2 launch neo_simulation2 simulation.launch.py my_robot:=mpo_700 world:=neo_workshop`

2. **Launch the Navigation Stack:**

   This command launches the navigation stack with the required configuration for the MPO 700:

   `ros2 launch neo_simulation2 navigation.launch.py use_sim_time:=True map:=neo_workshop param_file:=/home/username/ros2_humble_neobotix_workspace/src/neo_simulation2/configs/mpo_700/navigation.yaml`

   > Note: Make sure to adjust the path for the param_file according to your local setup.

3. **Launch RViz for Visualization:**

   This command brings up RViz for visualization purposes:

   `ros2 launch neo_nav2_bringup rviz_launch.py`

4. **Launch the Docking Server:**

   This command launches the docking server for the MPO 700:

   `ros2 launch neo_docking2 docking_launch.py`

   - The docking server includes a custom pose publisher that triggers when the robot reaches the staging pose. It reads the dock pose from the `dock_database.yaml` file.

## Saving the Dock Pose

To save the dock pose to the YAML file, call the following service:

   `ros2 service call /save_dock_pose_to_yaml neo_srvs2/srv/SaveDockPose "{dock_id: 'dock1', dock_type: 'simple_dock', dock_frame: 'map'}"`

This will save a YAML file named `dock_database.yaml` in the configuration directory.

Example position with dock pose in neo_workshop world:

```
docks:
  dock1:
    frame: map
    pose: [-5.094569517241989, -2.8147842123288074, -0.010333245175675873]
    type: simple_dock
```

## Docking and Undocking

To trigger the docking action, all the following action command. Provide the dock_id that matches the saved dock in the `dock_database.yaml`:

   `ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot "{'dock_id': 'dock1'}"`

The docking server will retry up to 3 times in case of failure. When the robot reaches the staging pose, the static dock pose will be published.
The graceful controller is currently tuned as follows:

```
controller:
  k_phi: 10.0
  k_delta: 10.0
```

To undock the robot:

Call the following action command to undock:

   `ros2 action send_goal /undock_robot opennav_docking_msgs/action/UndockRobot "{'dock_type': 'simple_dock', 'max_undocking_time': 1000}"`

Make sure the dock_type matches and sufficient undocking time is provided.

## Summary of Commands

- Launch Gazebo simulation:

   `ros2 launch neo_simulation2 simulation.launch.py my_robot:=mpo_700 world:=neo_workshop`

- Launch navigation:

   `ros2 launch neo_simulation2 navigation.launch.py use_sim_time:=True map:=neo_workshop param_file:=/home/adarsh/ros2_humble_neobotix_workspace/src/neo_simulation2/configs/mpo_700/navigation.yaml`

- Launch RViz:

   `ros2 launch neo_nav2_bringup rviz_launch.py`

- Launch docking server:

   `ros2 launch neo_docking2 docking_launch.py`

- Save dock pose to YAML:

   `ros2 service call /save_dock_pose_to_yaml neo_srvs2/srv/SaveDockPose "{dock_id: 'dock1', dock_type: 'simple_dock', dock_frame: 'map'}"`

- Dock robot:

   `ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot "{'dock_id': 'dock1'}"`

- Undock robot:

   `ros2 action send_goal /undock_robot opennav_docking_msgs/action/UndockRobot "{'dock_type': 'simple_dock', 'max_undocking_time': 1000}"`

## Notes

- Make sure to adjust the paths and configurations as per your workspace setup.

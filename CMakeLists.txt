cmake_minimum_required(VERSION 3.8)
project(neo_docking2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(neo_srvs2 REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(opennav_docking REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)

# Build the C++ node
add_executable(neo_docking2 src/neo_docking.cpp)
# Add libraries for pose_filter and neo_charging_dock
add_library(pose_filter SHARED src/pose_filter.cpp)
add_library(neo_charging_dock SHARED src/neo_charging_dock.cpp)

# Set common dependencies
set(COMMON_DEPENDENCIES
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  pluginlib
  opennav_docking_msgs
  opennav_docking_core
  opennav_docking
  yaml_cpp_vendor
)

# Link common dependencies to the libraries
ament_target_dependencies(pose_filter ${COMMON_DEPENDENCIES})
ament_target_dependencies(neo_charging_dock ${COMMON_DEPENDENCIES})

# Link pose_filter to neo_charging_dock
target_link_libraries(neo_charging_dock pose_filter)

# Link neo_charging_dock and pose_filter to the executable
ament_target_dependencies(neo_docking2
  ${COMMON_DEPENDENCIES}
  rclpy
  std_srvs
  neo_srvs2
)
target_link_libraries(neo_docking2 neo_charging_dock pose_filter)
pluginlib_export_plugin_description_file(opennav_docking_core plugins.xml)

# Set include directories and compile features for pose_filter
target_include_directories(pose_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_filter PUBLIC c_std_99 cxx_std_17)

# Set include directories and compile features for neo_charging_dock
target_include_directories(neo_charging_dock PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(neo_charging_dock PUBLIC c_std_99 cxx_std_17)

# Set include directories and compile features for neo_docking2
target_include_directories(neo_docking2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(neo_docking2 PUBLIC c_std_99 cxx_std_17)

# Install executables and libraries
install(TARGETS neo_docking2 pose_filter neo_charging_dock
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
scripts/save_pose_to_yaml.py
scripts/publish_pose.py
DESTINATION lib/${PROJECT_NAME}
)
      
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

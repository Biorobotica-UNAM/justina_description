cmake_minimum_required(VERSION 3.8)
project(justina_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(depth_to_pointcloud
  src/virtual_pointcloud.cpp
)

ament_target_dependencies(depth_to_pointcloud
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# ===========================================
# INSTALL DIRECTORIES AND FILES
# ===========================================

# Install URDF files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install meshes directory (if it exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/meshes)
  install(
    DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/meshes
  )
endif()

# Install rviz configuration (if it exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(
    DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
  )
endif()

# Install webots_env subdirectory (controllers, protos, worlds, launch ...)
install(DIRECTORY
  webots_env/controllers
  webots_env/launch
  webots_env/models
  webots_env/protos
  webots_env/rviz
  webots_env/urdf
  webots_env/worlds
  webots_env/scripts
  DESTINATION share/${PROJECT_NAME}/webots_env
)

install(PROGRAMS
  webots_env/scripts/robot_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  webots_env/scripts/set_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS depth_to_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/model)
  install(
    DIRECTORY model/
    DESTINATION share/${PROJECT_NAME}/model
  )
endif()

ament_package()
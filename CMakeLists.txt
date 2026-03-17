cmake_minimum_required(VERSION 3.8)
project(justina_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

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

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/model)
  install(
    DIRECTORY model/
    DESTINATION share/${PROJECT_NAME}/model
  )
endif()

# Finalize package
ament_package()
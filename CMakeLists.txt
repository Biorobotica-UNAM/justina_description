cmake_minimum_required(VERSION 3.8)
project(justina_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

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
  webots_env/scripts/set_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

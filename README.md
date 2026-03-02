# justina_description
This repository contains a ROS package with all the robot description files for the Justina robot, such as .xacro, .urdf, .dae, and textures.

In this version Real robot uses urdf/justina xacro files, while the simulation version implements proto files form webots software.

Use install.sh file to install all necessary dependencies.

To launch simulated version of the robot, use launch files located in webots_env folder that loads a low poly version of the robot, which can be used for faster rendering in Gazebo, reducing poly count from 334,489 to 12,372 polygons for all robot mesh.


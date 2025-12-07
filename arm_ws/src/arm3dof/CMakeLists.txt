cmake_minimum_required(VERSION 3.16)
project(arm3dof)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install non-code assets (models/worlds/launch)
install(DIRECTORY
  models
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  src/move_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
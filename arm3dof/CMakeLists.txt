cmake_minimum_required(VERSION 3.16)
project(arm3dof)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY
  models/
  launch/
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/move_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.5)
project(quadropted_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS  
)
install(PROGRAMS
  scripts/robot_controller_gazebo.py
  scripts/QuadrupedOdometryNode.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_pub.py
  scripts/robot_mode.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(on_track_sys_id)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(
  PROGRAMS
    src/on_track_sys_id.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
install(
  DIRECTORY on_track_sys_id
  DESTINATION lib/python3.8/site-packages
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)
# Export package information
ament_package()

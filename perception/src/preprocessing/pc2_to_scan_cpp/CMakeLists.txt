cmake_minimum_required(VERSION 3.8)
project(pc2_to_scan_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pc2_to_laserscan src/pc2_to_laserscan.cpp)
ament_target_dependencies(pc2_to_laserscan
  rclcpp
  sensor_msgs
)

install(TARGETS
  pc2_to_laserscan
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

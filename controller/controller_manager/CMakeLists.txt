cmake_minimum_required(VERSION 3.8)
project(controller_manager)

# -------------------- find_package --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(mpcc_ros REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

# -------------------- mux_controller 노드 --------------------
add_executable(mux_controller
  src/mux_controller.cpp
)

target_include_directories(mux_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mux_controller
  rclcpp
  std_msgs
  sensor_msgs
  ackermann_msgs
  f110_msgs
  mpcc_ros
)

# -------------------- l1_controller 노드 --------------------
add_executable(l1_controller
  src/l1_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../map/src/map.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../map/src/lookup_steer_angle.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../pp/src/pp.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../ftg/src/ftg.cpp
)

target_include_directories(l1_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../map/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../pp/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ftg/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(l1_controller
  rclcpp
  std_msgs
  sensor_msgs
  ackermann_msgs
  f110_msgs
  mpcc_ros
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
  yaml-cpp
  rcl_interfaces
)

# yaml-cpp는 보통 이렇게 직접 링크 필요
target_link_libraries(l1_controller
  yaml-cpp
)

# -------------------- install --------------------
install(TARGETS
  mux_controller
  l1_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

add_compile_options(-Wno-psabi)

cmake_minimum_required(VERSION 3.8)
project(cluster_to_obstacle_cpp)

# C++ 표준
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(tier4_perception_msgs REQUIRED)
find_package(f110_msgs REQUIRED)

# Frenet 변환 (패키지명: frenet_conversion_cpp)
find_package(frenet_conversion_cpp REQUIRED)

# 실행 파일
add_executable(cluster_to_obstacle_node
  src/cluster_to_obstacle_node.cpp
)

# include 디렉토리 (로컬 include/가 있으면 유지)
target_include_directories(cluster_to_obstacle_node PRIVATE
  include
)

# 타깃 의존성
ament_target_dependencies(cluster_to_obstacle_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tier4_perception_msgs
  f110_msgs
  frenet_conversion_cpp
)

# 바이너리 설치
install(TARGETS
  cluster_to_obstacle_node
  DESTINATION lib/${PROJECT_NAME}
)

# 런치 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

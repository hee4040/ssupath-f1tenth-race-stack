cmake_minimum_required(VERSION 3.8)
project(tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------- Find packages --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ★ 네가 제공한 프레네트 변환 패키지
find_package(frenet_conversion_cpp REQUIRED)

# -------------------- Build target --------------------
add_executable(tracking src/tracking.cpp)

ament_target_dependencies(tracking
  rclcpp
  nav_msgs
  sensor_msgs
  visualization_msgs
  f110_msgs
  tf2
  tf2_geometry_msgs
  frenet_conversion_cpp
)

# Eigen 링크
target_link_libraries(tracking Eigen3::Eigen)

# -------------------- Install --------------------
install(TARGETS
  tracking
  DESTINATION lib/${PROJECT_NAME})

# (선택) launch 디렉토리 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

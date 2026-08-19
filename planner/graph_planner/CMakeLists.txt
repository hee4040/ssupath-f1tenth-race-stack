cmake_minimum_required(VERSION 3.8)
project(graph_planner)

# C++17 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-psabi")

# 외부 라이브러리
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)  # matplotlib-cpp 필요

# ROS2 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(frenet_conversion_cpp REQUIRED)

# 소스 파일
set(SOURCES
    src/graph_planner.cpp
    src/graph_planner_utils.cpp
    src/offline_params.cpp
    src/visualization.cpp
)

# 실행 파일
add_executable(graph_planner ${SOURCES})

# include 디렉토리 지정 
target_include_directories(graph_planner
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../utilities/libraries/frenet_conversion_cpp/include
)

# 의존성 연결
ament_target_dependencies(graph_planner
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    f110_msgs
    frenet_conversion_cpp
)

# yaml-cpp 링크
target_link_libraries(graph_planner
    yaml-cpp
)

# matplotlib-cpp (헤더만 있는 라이브러리)
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp INTERFACE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/matplotlib-cpp   # matplotlibcpp.h 경로
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(matplotlib_cpp INTERFACE
    Python3::Python
    Python3::Module
)

# graph_planner 실행파일에 matplotlib-cpp 연결
target_link_libraries(graph_planner
    matplotlib_cpp
)

# 실행 파일 설치
install(TARGETS graph_planner
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 패키지 설정
ament_package()

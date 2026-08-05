cmake_minimum_required(VERSION 3.8)
project(mpcc_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(OpenMP REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SHint.msg"
  "msg/Corridor.msg"
  "msg/MpccControl.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


set(MPCC_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../MPCC/C++)
set(HPIPM_LIB   ${MPCC_SRC_DIR}/External/hpipm/lib/lib/libhpipm.a)
set(BLASFEO_LIB ${MPCC_SRC_DIR}/External/blasfeo/lib/lib/libblasfeo.a)

include_directories(
  ${MPCC_SRC_DIR}
  ${MPCC_SRC_DIR}/Constraints
  ${MPCC_SRC_DIR}/Cost
  ${MPCC_SRC_DIR}/Interfaces
  ${MPCC_SRC_DIR}/MPC
  ${MPCC_SRC_DIR}/Model
  ${MPCC_SRC_DIR}/Params
  ${MPCC_SRC_DIR}/Spline

  ${MPCC_SRC_DIR}/External/blasfeo/lib/include
  ${MPCC_SRC_DIR}/External/hpipm/lib/include
  ${MPCC_SRC_DIR}/External/Eigen
  ${MPCC_SRC_DIR}/External/Json/include
  ${MPCC_SRC_DIR}/External/matplotlib
  ${Python3_INCLUDE_DIRS}
)

file(GLOB_RECURSE MPCC_SOURCES
  ${MPCC_SRC_DIR}/MPC/*.cpp
  ${MPCC_SRC_DIR}/Model/*.cpp
  ${MPCC_SRC_DIR}/Params/*.cpp
  ${MPCC_SRC_DIR}/Constraints/*.cpp
  ${MPCC_SRC_DIR}/Cost/*.cpp
  ${MPCC_SRC_DIR}/Spline/*.cpp
  ${MPCC_SRC_DIR}/Interfaces/*.cpp
  ${MPCC_SRC_DIR}/Plotting/*.cpp
  ${MPCC_SRC_DIR}/types.cpp
)


add_executable(mpcc_node
  src/mpcc_node.cpp
  ${MPCC_SOURCES}
)

target_compile_options(mpcc_node PRIVATE $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-fopenmp>)
set_target_properties(mpcc_node PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
)

ament_target_dependencies(mpcc_node
  rclcpp nav_msgs geometry_msgs ackermann_msgs
  tf2 tf2_ros tf2_geometry_msgs
  vesc_msgs Eigen3 visualization_msgs std_msgs
  rosidl_default_runtime
)

target_link_libraries(mpcc_node
  ${HPIPM_LIB}
  ${BLASFEO_LIB}
  Python3::Python
  OpenMP::OpenMP_CXX
  m
  "${cpp_typesupport_target}"
)


add_executable(mpcc_node_sim
  src/mpcc_node_sim.cpp
  ${MPCC_SOURCES}
)

target_compile_options(mpcc_node_sim PRIVATE $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-fopenmp>)
set_target_properties(mpcc_node_sim PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
)


ament_target_dependencies(mpcc_node_sim
  rclcpp nav_msgs geometry_msgs ackermann_msgs
  tf2 tf2_ros tf2_geometry_msgs
  vesc_msgs Eigen3 visualization_msgs std_msgs
  rosidl_default_runtime
)

target_link_libraries(mpcc_node_sim
  ${HPIPM_LIB}
  ${BLASFEO_LIB}
  Python3::Python
  OpenMP::OpenMP_CXX
  m
  "${cpp_typesupport_target}"
)


add_executable(dummy_corridor_generator
  src/dummy_corridor_generator.cpp
)

target_compile_features(dummy_corridor_generator PUBLIC cxx_std_17)

ament_target_dependencies(dummy_corridor_generator
  rclcpp
  rosidl_default_runtime
)

target_link_libraries(dummy_corridor_generator
  "${cpp_typesupport_target}"
)


install(TARGETS
  mpcc_node
  mpcc_node_sim
  dummy_corridor_generator
  DESTINATION lib/${PROJECT_NAME}
)

# config 파일 설치
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

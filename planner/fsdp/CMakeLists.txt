cmake_minimum_required(VERSION 3.8)
project(fsdp)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OTSection.msg"
  "msg/OTSections.msg"
  "msg/ProjOppPoint.msg"
  "msg/ProjOppTraj.msg"
  DEPENDENCIES std_msgs
)

install(FILES
  src/fsdp/ros_compat.py
  DESTINATION local/lib/python3.10/dist-packages/fsdp
)

install(DIRECTORY
  src/common
  src/gp
  src/mpc
  src/soc
  src/sqp
  src/waypoint
  DESTINATION local/lib/python3.10/dist-packages
  FILES_MATCHING PATTERN "*.py"
)

add_custom_target(${PROJECT_NAME}_mpc_builder ALL
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/setup.py build_ext --inplace
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Building fsdp mpc_builder Python extension"
)


# 확장 모듈 파일명은 플랫폼마다 다르다 (x86_64 / aarch64 ...).
# install 시점에 glob 되도록 install(DIRECTORY)로 처리한다.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/
  DESTINATION local/lib/python3.10/dist-packages
  FILES_MATCHING PATTERN "mpc_builder*.so"
)

install(PROGRAMS
  src/soc/collision_prediction.py
  src/soc/dynamic_collision_server.py
  src/sqp/dynamic_sqp_server.py
  src/sqp/sqp_avoidance_node.py
  src/waypoint/update_waypoints.py
  src/gp/gaussian_process_opp_traj.py
  src/gp/opponent_trajectory.py
  src/gp/predictor_opponent_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

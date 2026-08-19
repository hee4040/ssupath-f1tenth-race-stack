cmake_minimum_required(VERSION 3.14)
project(autoware_pointcloud_preprocessor)

find_library(TINYXML2_LIBRARY tinyxml2 REQUIRED)
find_path(TINYXML2_INCLUDE_DIR tinyxml2.h REQUIRED)

if(NOT TARGET tinyxml2::tinyxml2)
  add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
  set_target_properties(tinyxml2::tinyxml2 PROPERTIES
    IMPORTED_LOCATION "${TINYXML2_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIR}"
  )
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(autoware_point_types REQUIRED)
find_package(autoware_utils REQUIRED)
find_package(managed_transform_buffer REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

# ========== Filter Base Library (추가) ==========
ament_auto_add_library(filter_base SHARED
  src/filter.cpp                 # ← Filter 추상 클래스 구현
  src/utility/memory.cpp         # ← Autoware Filter 유틸 (필요하면)
)

ament_target_dependencies(filter_base
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  autoware_point_types
  autoware_utils
  managed_transform_buffer
  tf2_ros
)

# ========== Passthrough Filter Library ==========
ament_auto_add_library(passthrough_filter SHARED
  src/passthrough_filter/passthrough_filter_node.cpp
  src/passthrough_filter/passthrough_filter_uint16_node.cpp
  src/passthrough_filter/passthrough_uint16.cpp
)

ament_target_dependencies(passthrough_filter
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  autoware_point_types
  autoware_utils
  managed_transform_buffer
  tf2_ros
)

# 여기서 filter_base를 passthrough_filter에 링크
target_link_libraries(passthrough_filter
  filter_base
)

# ========== Component 등록 ==========
rclcpp_components_register_node(passthrough_filter
  PLUGIN "autoware::pointcloud_preprocessor::PassThroughFilterComponent"
  EXECUTABLE passthrough_filter_node)

rclcpp_components_register_node(passthrough_filter
  PLUGIN "autoware::pointcloud_preprocessor::PassThroughFilterUInt16Component"
  EXECUTABLE passthrough_filter_uint16_node)

# ========== Install ==========
install(
  TARGETS filter_base passthrough_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

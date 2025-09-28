cmake_minimum_required(VERSION 3.8)
project(pointcloud2_cutter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(pointcloud2_cutter_lib
  src/region_selector.cpp
  src/pointcloud_filter.cpp
  src/pointcloud2_cutter_node.cpp
)

target_include_directories(pointcloud2_cutter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(pointcloud2_cutter_lib PUBLIC cxx_std_17)

ament_target_dependencies(pointcloud2_cutter_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(pointcloud2_cutter_lib yaml-cpp)

add_executable(pointcloud2_cutter_node
  src/pointcloud2_cutter.cpp
)

target_compile_features(pointcloud2_cutter_node PRIVATE c_std_99 cxx_std_17)

ament_target_dependencies(pointcloud2_cutter_node
  rclcpp
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(pointcloud2_cutter_node
  pointcloud2_cutter_lib
)

install(TARGETS
  pointcloud2_cutter_lib
  pointcloud2_cutter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_RESOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test/resources)

  ament_add_gtest(test_region_selector test/test_region_selector.cpp)
  target_link_libraries(test_region_selector pointcloud2_cutter_lib)
  target_compile_definitions(test_region_selector PRIVATE TEST_RESOURCES_DIR="${TEST_RESOURCES_DIR}")

  ament_add_gtest(test_pointcloud_filter test/test_pointcloud_filter.cpp)
  target_link_libraries(test_pointcloud_filter pointcloud2_cutter_lib)
  target_compile_definitions(test_pointcloud_filter PRIVATE TEST_RESOURCES_DIR="${TEST_RESOURCES_DIR}")
endif()

ament_package()

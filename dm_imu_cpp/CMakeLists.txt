cmake_minimum_required(VERSION 3.8)
project(dm_imu_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(dm_serial src/dm_serial.cpp)
target_include_directories(dm_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(dm_serial PUBLIC cxx_std_17)

add_executable(dm_imu_node src/dm_imu_node.cpp)
target_include_directories(dm_imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(dm_imu_node PUBLIC cxx_std_17)

target_link_libraries(dm_imu_node dm_serial)

ament_target_dependencies(dm_imu_node
  rclcpp
  sensor_msgs
  geometry_msgs)

install(TARGETS dm_serial dm_imu_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

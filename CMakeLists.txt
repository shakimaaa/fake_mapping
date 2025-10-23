cmake_minimum_required(VERSION 3.8)
project(local_btmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(local_btmaplib SHARED
  src/localmap_from_bt.cpp
)

target_link_libraries(local_btmaplib
  ${EIGEN3_LIBRARIES}
  -loctomap
  -loctomath
)

ament_target_dependencies(local_btmaplib
  rclcpp
  sensor_msgs
  nav_msgs
  octomap_msgs
  octomap
  std_msgs
)

add_executable(local_btmap_node src/main.cpp)
target_link_libraries(local_btmap_node local_btmaplib -loctomath)

ament_target_dependencies(local_btmap_node
  rclcpp
  sensor_msgs
  nav_msgs
  octomap_msgs
  octomap
  std_msgs
)

install(TARGETS local_btmaplib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS local_btmap_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)
ament_export_include_directories(include)
ament_export_libraries(local_btmaplib)
ament_export_dependencies( rclcpp sensor_msgs  nav_msgs  octomap_msgs  octomap  std_msgs)


ament_package()

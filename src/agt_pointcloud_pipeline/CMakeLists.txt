cmake_minimum_required(VERSION 3.8)
project(agt_pointcloud_pipeline)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(agt_pointcloud_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME}_plugins SHARED src/builtin_plugins.cpp)
target_include_directories(${PROJECT_NAME}_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_plugins
  agt_pointcloud_core rclcpp pluginlib)

add_executable(filter_node src/filter_node.cpp)
target_include_directories(filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(filter_node
  agt_pointcloud_core rclcpp sensor_msgs std_msgs tf2 tf2_ros pluginlib)

pluginlib_export_plugin_description_file(agt_pointcloud_pipeline plugins.xml)

install(TARGETS ${PROJECT_NAME}_plugins filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(agt_pointcloud_core rclcpp pluginlib)
ament_package()

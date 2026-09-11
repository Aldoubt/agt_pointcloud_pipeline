cmake_minimum_required(VERSION 3.8)
project(agt_pointcloud_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_filters test/test_filters.cpp)
  target_link_libraries(test_filters ${PROJECT_NAME})
endif()

ament_package()

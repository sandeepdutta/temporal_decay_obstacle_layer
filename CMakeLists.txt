cmake_minimum_required(VERSION 3.16)
project(jeeves_temporal_obstacle_layer)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release optimizations
if(CMAKE_BUILD_TYPE STREQUAL Release)
  add_compile_options(-O3 -march=native -DNDEBUG)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(
  include
  ${nav2_costmap_2d_INCLUDE_DIRS}
)

# Library
add_library(jeeves_temporal_obstacle_layer_lib SHARED
  src/temporal_obstacle_layer.cpp
)

target_link_libraries(jeeves_temporal_obstacle_layer_lib
  ${nav2_costmap_2d_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

ament_target_dependencies(jeeves_temporal_obstacle_layer_lib
  nav2_costmap_2d
  rclcpp
  sensor_msgs
  pluginlib
  geometry_msgs
  tf2_geometry_msgs
  tf2_sensor_msgs
)

# Plugin registration
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS jeeves_temporal_obstacle_layer_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # Unit tests
  ament_add_gtest(test_temporal_obstacle_layer
    test/test_temporal_obstacle_layer.cpp
  )

  if(TARGET test_temporal_obstacle_layer)
    target_link_libraries(test_temporal_obstacle_layer
      jeeves_temporal_obstacle_layer_lib
      ${nav2_costmap_2d_LIBRARIES}
      ${rclcpp_LIBRARIES}
      ${sensor_msgs_LIBRARIES}
    )
  endif()
endif()

ament_package()

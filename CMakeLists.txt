cmake_minimum_required(VERSION 3.5)
project(mappi)

add_compile_options(-std=c++14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")

# add_definitions(-DXTENSOR_ENABLE_XSIMD)
# add_definitions(-DXTENSOR_USE_XSIMD)

set(XTENSOR_USE_TBB 1)
set(XTENSOR_USE_OPENMP 0)

find_package(xtensor REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_ros_common REQUIRED)

nav2_package()

include_directories(
  include
  mappi/include
  ${xtensor_INCLUDE_DIRS}
)

# MPPIc (standalone lib)
add_library(${PROJECT_NAME} 
            mappi/src/mppic.cpp
            )
target_compile_options(${PROJECT_NAME} PUBLIC)
target_include_directories(${PROJECT_NAME} PUBLIC ${xtensor_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} xtensor nav2_costmap_2d)

# ROS MPPIc library (nav2 controller)
add_library(${PROJECT_NAME}_nav2_controller SHARED
              src/Controller.
              cpp
              )
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME}_nav2_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_link_libraries(${PROJECT_NAME}_nav2_controller PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${PROJECT_NAME}
  nav2_core::nav2_core
  nav2_costmap_2d::filters
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_ros_common::nav2_ros_common
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2::tf2
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  nav2_ros_common
  visualization_msgs
  std_srvs
  std_msgs
  ${PROJECT_NAME}
)

# Binaries
install(TARGETS ${PROJECT_NAME}_nav2_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files.
install(DIRECTORY
  # launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_nav2_controller)
ament_export_dependencies(${dependencies})

# Export plugin
pluginlib_export_plugin_description_file(nav2_core mappi_plugin.xml)

ament_package()
cmake_minimum_required(VERSION 3.5)
project(mappi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# add_definitions(-DXTENSOR_ENABLE_XSIMD)
# add_definitions(-DXTENSOR_USE_XSIMD)

set(XTENSOR_USE_TBB 1)
set(XTENSOR_USE_OPENMP 0)
# set(XTENSOR_USE_XSIMD 1)

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

nav2_package()

set(dependencies_pkgs
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  nav2_common
  tf2
  visualization_msgs
  std_srvs
  std_msgs
)

include_directories(
  include
  mappi/include
  ${xtensor_INCLUDE_DIRS}
)

# MPPIc (standalone lib)
add_library(${PROJECT_NAME} 
            mappi/src/Critics/Forward.cpp
            mappi/src/Critics/Goal.cpp
            mappi/src/Critics/GoalAngle.cpp
            mappi/src/Critics/Obstacles.cpp
            mappi/src/Critics/PathAngle.cpp
            mappi/src/Critics/PathDist.cpp
            mappi/src/Critics/PathFollow.cpp
            mappi/src/Critics/Twirling.cpp
            mappi/src/Utils/NoiseGenerator.cpp
            mappi/src/Utils/Splines/BaseSpline.cpp
            mappi/src/Utils/Splines/CubicBSpline.cpp
            mappi/src/Utils/Filters/LowPass.cpp
            mappi/src/Utils/Filters/SavitskyGolay.cpp
            mappi/src/mppic.cpp
            )
target_compile_options(${PROJECT_NAME} PUBLIC)
target_include_directories(${PROJECT_NAME} PUBLIC ${xtensor_INCLUDE_DIRS}) # ${xsimd_INCLUDE_DIRS}
target_link_libraries(${PROJECT_NAME} xtensor) # xtensor::optimize xtensor::use_xsimd

# ROS Nav2 controller plugin (depends on nav2 + mappi core lib)
add_library(${PROJECT_NAME}_nav2_controller SHARED
              src/Controller.cpp
              src/ParametersHandler.cpp
              )
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME}_nav2_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(${PROJECT_NAME}_nav2_controller ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_nav2_controller ${dependencies_pkgs})

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
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_nav2_controller)
ament_export_dependencies(${dependencies_pkgs})

# Export plugin
pluginlib_export_plugin_description_file(nav2_core mappi_plugin.xml)

ament_package()
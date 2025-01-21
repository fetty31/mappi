cmake_minimum_required(VERSION 3.0.2)
project(nano_mppic)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")

# add_definitions(-DXTENSOR_ENABLE_XSIMD)
# add_definitions(-DXTENSOR_USE_XSIMD)

set(XTENSOR_USE_TBB 0)
set(XTENSOR_USE_OPENMP 0)

find_package(xtensor REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  dynamic_reconfigure
  costmap_2d
  nav_core
  tf2
)

generate_dynamic_reconfigure_options(
  cfg/NanoMPPIc.cfg
)

catkin_package(
 INCLUDE_DIRS include nano_mppic/include
 LIBRARIES ${PROJECT_NAME}_local_planner 
 CATKIN_DEPENDS roscpp nav_msgs tf2 costmap_2d geometry_msgs nav_core visualization_msgs std_msgs sensor_msgs
#  DEPENDS XTENSOR
)

include_directories(
  include
  nano_mppic/include
  ${catkin_INCLUDE_DIRS}
  ${xtensor_INLCUDE_DIRS}
)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

# Nano MPPIC (standalone lib)
add_library(${PROJECT_NAME} 
            nano_mppic/src/mppic.cpp
            )
target_compile_options(${PROJECT_NAME} PUBLIC)
target_include_directories(${PROJECT_NAME} PUBLIC ${xtensor_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} xtensor ${catkin_LIBRARIES})

# ROS nano MPPIC library
add_library(${PROJECT_NAME}_local_planner
              src/Controller.cpp
              )

add_dependencies(${PROJECT_NAME}_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_local_planner ${PROJECT_NAME} ${catkin_LIBRARIES})

# Binaries
install(TARGETS ${PROJECT_NAME}_local_planner
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES
  nano_mppic_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
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
  costmap_2d
  tf2
)

catkin_package(
 INCLUDE_DIRS include nano_mppic/include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp nav_msgs tf2 costmap_2d
#  DEPENDS XTENSOR
)

include_directories(
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

# Nano MPPIC
add_library(${PROJECT_NAME} 
            nano_mppic/src/Predictor.cpp
            )
target_compile_options(${PROJECT_NAME} PUBLIC)
target_include_directories(${PROJECT_NAME} PUBLIC ${xtensor_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} xtensor ${catkin_LIBRARIES})

# # ROS execs
# add_executable(${PROJECT_NAME}_exec
#               src/Controller.cpp
#               )

# add_dependencies(${PROJECT_NAME}_exec ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_exec ${PROJECT_NAME} ${catkin_LIBRARIES})

# # Binaries
# install( TARGETS ${PROJECT_NAME}_exec
#          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
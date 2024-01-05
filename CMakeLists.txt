cmake_minimum_required(VERSION 3.0.2)
project(my_fastlio2)
# cmake_policy(SET CMP0048 NEW) # 去除一些警告

SET( CMAKE_CXX_FLAGS "-std=c++14 -O3" ) # -O3 表示进行高级优化
add_compile_options(-std=c++14)

# 使用 ROOT_DIR 这个宏来表示当前源代码目录的路径
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DCONFIG_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/config/\")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

# # 去除很多由PCL库导致的warning
# if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
#      set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
# endif()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_fastlio2
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  include/my_fastlio2
  include/wrapper
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

SET(RONTEND_FILE
    src/my_fastlio2/modules/frontend.cpp
    src/my_fastlio2/modules/map_manager.cpp
    src/my_fastlio2/modules/state_manager.cpp
)
SET(FRONTEND_WRAPPER_FILE src/wrapper/frontend_wrapper.cpp)

add_executable(ieskf_frontend_node src/app/my_fastlio2.cpp
${RONTEND_FILE}
${FRONTEND_WRAPPER_FILE})

target_link_libraries(ieskf_frontend_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
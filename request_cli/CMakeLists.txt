cmake_minimum_required(VERSION 3.0.2)
project(request_cli)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES request_cli
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


# 添加可执行文件
add_executable(keyboard_planning_request src/keyboard_planning_request.cpp)
add_executable(auto_planning src/auto_planning.cpp)
add_executable(fast src/fast_planning.cpp)

# 链接库
target_link_libraries(keyboard_planning_request
  ${catkin_LIBRARIES}
)
target_link_libraries(auto_planning
  ${catkin_LIBRARIES}
)

target_link_libraries(fast
  ${catkin_LIBRARIES}
)
cmake_minimum_required(VERSION 3.0.2)
project(z1_hw)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  control_msgs
  actionlib_msgs
  actionlib
  z1_sdk
  position_controllers #新增
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    controller_manager 
    hardware_interface 
    roscpp 
    control_msgs 
    actionlib_msgs
    z1_sdk
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${z1_sdk_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_node src/z1_hw.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${z1_sdk_LIBRARIES})

# 在CMakeLists.txt中添加
add_executable(high_freq_control src/high_freq_control.cpp)
target_link_libraries(high_freq_control ${catkin_LIBRARIES})
add_executable(joint1_high_freq_controller src/joint1_high_freq_controller.cpp)
target_link_libraries(joint1_high_freq_controller ${catkin_LIBRARIES})
add_executable(high_freq_control_single src/high_freq_control_single.cpp) #从 high_freq_control 固定下来的稳定版本
target_link_libraries(high_freq_control_single ${catkin_LIBRARIES})
add_executable(high_freq_control_test src/high_freq_control_test.cpp) #从 high_freq_control 修改而来的性能测试版本
target_link_libraries(high_freq_control_test ${catkin_LIBRARIES})
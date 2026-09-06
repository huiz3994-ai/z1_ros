cmake_minimum_required(VERSION 3.1.3)
project(z1_examples)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometry_msgs
    tf2_ros   # 新增
    tf2_eigen # 新增
    tf2_geometry_msgs  # 平滑控制计算必须添加
    trac_ik_lib   # 新增
    unitree_legged_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(NLopt REQUIRED)  # 新增
find_package(PCL REQUIRED) # 新增，用于可视化

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM
 ${Boost_INCLUDE_DIRS}
 ${z1_sdk_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
 ${Eigen3_INCLUDE_DIRS} # 新增
 ${NLopt_INCLUDE_DIRS}  # 新增
 ${PCL_INCLUDE_DIRS}    # 新增，用于可视化
 include
)

link_directories(       # 新增，用于可视化
  ${PCL_LIBRARY_DIRS}   # 新增，用于可视化
)                       # 新增，用于可视化

# examples
set(EXAMPLE_FILES
  examples/ros_gripper_ctrl.cpp
  examples/move_group_interface.cpp
  examples/get_end_effector_pose.cpp #从 test2 固定下来的完善版本
  examples/real_time_ik_solver_single.cpp   #从 test3 固定下来的稳定版本
  examples/gripper_ctrl_mycode.cpp
  examples/gripper_ctrl2_mycode.cpp
  examples/gripper_ctrl2_mycode_copy.cpp
  examples/gripper_ctrl2_mycode_copy3.cpp
  examples/gripper_ctrl2_mycode_copy4.cpp
  
  examples/test1_Touch_PLUS_vision_solver.cpp
  examples/test2_get_end_effector_pose.cpp
  examples/test3_real_time_ik_solver.cpp
  examples/test4_real_time_fk_solver.cpp
  examples/test5_Joint_limits_check.cpp
  examples/test7_visial_servo_tele.cpp
  examples/test8_visial_servo_solver2.cpp

  examples/testD1T1_Touch_PLUS_vision_solver.cpp
  examples/testD1T2_Touch_PLUS_vision_solver.cpp

  examples/Htest1_Touch_PLUS_vision_solver.cpp
)

foreach(EXAMPLE_FILE IN LISTS EXAMPLE_FILES)
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
  target_link_libraries(${EXAMPLE_NAME} ${catkin_LIBRARIES} )
endforeach()

# 新增，用于可视化
add_executable(test6_workspace_check_visualizable examples/test6_workspace_check_visualizable.cpp)
target_link_libraries(test6_workspace_check_visualizable
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

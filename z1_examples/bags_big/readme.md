# 推荐录包命令如下：

rosbag record -O exp_${MODE}_$(date +%Y%m%d_%H%M%S).bag \
/Geomagic/pose \
/Geomagic/joy \
/Touch_Pose_increment \
/Touch_Anchor_updated \
/aruco_single/pose \
/aruco_single/aruco_detected_flag \
/detected_plane_pose_link00 \
/back_to_home_arm \
/back_to_zero_arm \
/ttttttttarget_pose \
/debug/current_pose \
/debug/k_aruco_pos_ctrl \
/debug/ctrl_mode \
/debug/ik_solved \
/debug/ik_solve_time \
/debug/processed_aruco_pose \
/debug/processed_realsense_pose \
/debug/anchor_pose \
/goal_joint_positions \
/joint_group_position_controller/command \
/joint_states \
/debug/high_freq/goal_positions \
/debug/high_freq/current_values \
/debug/high_freq/actual_joint_positions \
/debug/high_freq/all_reached \
/end_effector_pose \
/tf \
/tf_static


/camera/aligned_depth_to_color/camera_info \
/camera/aligned_depth_to_color/image_raw \
/camera/color/camera_info \
/camera/color/image_raw \

# 建议同时导出参数，不然 bag 里没有参数服务器内容：

rosparam dump exp_${MODE}_$(date +%Y%m%d_%H%M%S)_params.yaml

# 推荐回放命令：

rosparam set use_sim_time true
rosbag play exp_xxx.bag --clock --pause

# 回放结束后恢复：

rosparam set use_sim_time false

# Word 文档里已经整理好了：
代码改动说明、最终录包与回放命令、每个话题的作用说明、以及按 only_touch / aruco_for_pos / aruco_for_ori / realsense_for_ori / mixed_pos 分模式的绘图建议。
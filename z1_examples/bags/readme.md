# bag录制命令：
rosbag record --lz4 -O mixed_pos_trial_01.bag /Touch_Pose_increment /aruco_single/aruco_detected_flag /ttttttttarget_pose /end_effector_pose /goal_joint_positions /Geomagic/joy /Geomagic/pose /Touch_Anchor_updated /back_to_home_arm /back_to_zero_arm /joint_states /debug/k_aruco_pos_ctrl /debug/current_pose


# bag分析命令(在/home/v233/z1_ws/src/z1_ros/z1_examples/bags文件夹下)：
python3 process_mixed_pos_bag.py --bag /home/v233/z1_ws/src/z1_ros/z1_examples/bags/mixed_pos_exp/mixed_pos_trial_06.bag --outdir mixed_pos_figs_06 --k-touch 0.001

# bag回放：
## 循环回放，按 空格 暂停，按 s 步进（启动时默认暂停）：
rosbag play -l -r 0.5  --clock /home/v233/z1_ws/src/z1_ros/z1_examples/bags/mixed_pos_exp/mixed_pos_trial_04.bag --pause
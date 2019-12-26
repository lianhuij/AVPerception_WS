### node : lidar_cluster_node

receive topic : velodyne_points

publish topic :  
- no_ground_pc
- lidar_raw_rviz
- lidar_rawArray
- ego_car

---
### node : tracker_node

receive topic :  
- radar_rawArray
- camera_rawArray
- lidar_rawArray

publish topic :  
- radar_cmkf_rviz
- camera_kf_rviz
- lidar_kf_rviz
- fusion_rviz 
- radar_array
- lidar_array
- fusion_array

---
### node : rear_tracker_node

receive topic :  
- left_radar_rawArray
- right_radar_rawArray
- left_ultrasonic_raw
- right_ultrasonic_raw

publish topic :  
- left_radar_rviz
- right_radar_rviz
- left_radar_array
- right_radar_array 
- left_ultrasonic
- right_ultrasonic
- radar_fusion_array

### save bag data
- rosbag record -O file.bag radar_rawArray camera_rawArray lidar_rawArray radar_array lidar_array fusion_array
- rosbag record -O file.bag -a
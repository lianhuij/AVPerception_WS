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
### save bag data
rosbag record -O file radar_rawArray camera_rawArray lidar_rawArray radar_array lidar_array fusion_array
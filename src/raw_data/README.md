### node : MPC_raw_node

receive topic : received_messages

publish topic :  
- radar_raw_rviz
- camera_raw_rviz
- radar_rawArray
- camera_rawArray 
---
### node : rear_raw_node

receive topic : received messages

publish topic :
- right_radar_raw_rviz
- left_radar_raw_rviz
- right_radar_rawArray
- left_radar_rawArray
- left_ultrasonic_raw
- right_ultrasonic_raw
---
### connect kvaser can
sudo modprobe kvaser_usb  
sudo ip link set can0 up type can bitrate 500000

---
### save bag data
rosbag record -O file.bag velodyne_points received_messages  
rosbag record -O file.bag received_messages
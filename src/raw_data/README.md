node : MPC_raw_node

receive topic : received messages

publish topic :  
- radar_raw_rviz
- cam_raw_rviz
- radar_rawArray
- cam_rawArray 
---
node : rear_raw_node

receive topic : received messages

publish topic :
- rear_radar_raw_rviz
- rear_radar_rawArray

#### connect kvaser can
sudo modprobe kvaser_usb
sudo ip link set can0 up type can bitrate 500000


#### save bag data
rosbag record -O <file> velodyne_points received_messages
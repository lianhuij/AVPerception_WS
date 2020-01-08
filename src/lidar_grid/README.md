# lidar_grid
velodyne VLP16 point cloud processing
---
drivable area extraction method include:  
RANSAC  
gradient method
---
### node : azimuth_calibration

receive topic : velodyne_points

publish topic : azi_pc
---
### node : ego_car_node

publish topic : ego_car
---
### node : lidar_calibration

receive topic : velodyne_points

publish topic : 
- cali_pc
- cali_time

---
### node : lidar_grid

receive topic : cali_pc

publish topic : 
- grid_cell
- time
- ground_z

---
### node : lidar_grid2

receive topic : cali_pc

publish topic : 
- grid_cell
- time

---
### node : pcd_read

publish topic : velodyne_points
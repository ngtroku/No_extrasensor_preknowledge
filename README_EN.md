# Overview
An extra-sensor-free and prior-agnostic defense method that detects and mitigates point cloud tampering caused by LiDAR spoofing using only point cloud spatial information and scan matching-based ego-motion estimation. Against random noise injection attacks, this method suppresses the absolute trajectory error (ATE) of LiDAR SLAM by **up to 32% in simulation** and **20% in real-world environments**. This work is presented at the Institute of Electronics, Information and Communication Engineers (IEICE) Society Conference (2026). 
![Pipeline](images/git_header_fig.png)

# Prerequisites & Environment 
**OS:** Ubuntu 20.04 (with ROS Noetic) 
**Python:** 3.10.20 
**ROS:** ROS Noetic (Implementation for ROS2 is planned in future releases) 
## Dependencies 
- `numpy` 
-  `open3d` 
-  `rosbags` 
-  `scikit-learn` 
- `scipy`
-  `small_gicp` ([GitHub Repository](https://github.com/koide3/small_gicp)) 
---
## Installation 
```bash 
# Clone the repository
git clone https://github.com/ngtroku/No_extrasensor_preknowledge.git(https://github.com/ngtroku/no_extra_def.git) 
cd No_extrasensor_preknowledge 
# Install required Python packages 
pip install -r requirements.txt
```

# Usage and Config settings
Configure the pipeline parameters in `config.json` before running the mitigation script.
### 1. `general_settings`
- `input_rosbag`: Path to the source ROS1 rosbag.
- `lidar_topic`: The ROS topic name for the LiDAR point cloud (`sensor_msgs/msg/PointCloud2`).
- `topic_length`: The point cloud encoding format. Refer to the `point_step` value by running `rostopic echo /lidar_topic --noarr`.
- `lidar_only`: Set to `true` if the `input_rosbag` does not contain IMU topics. Otherwise, set to `false`.
- `imu_topic`: The ROS topic name for the IMU (`sensor_msgs/msg/Imu`).
### 2. `generate_pointcloud`
- `spoofer_x` / `spoofer_y`: The X and Y coordinates where the spoofing device is deployed.
- `horizontal_resolution` / `vertical_lines`: The horizontal resolution (deg) and the number of vertical channels of the targeted LiDAR sensor.
### 3. `spoofed_rosbag_params`
- `output_rosbag`: Output path for the simulated spoofed rosbag.
### 4. `mitigate_params`
- `output_rosbag`: Output path for the cleaned rosbag processed by the proposed method.
## License
This project is licensed under the MIT License - see the [LICENSE](https://www.google.com/search?q=LICENSE) file for details.

# 概要
点群情報, スキャンマッチングによる姿勢推定のみを用いてLiDAR Spoofingによる点群改ざんを検出・除去する手法. ランダムノイズ注入攻撃に対し, シミュレーションにて最大32%, 実世界においても20% LiDAR SLAMに生じる位置ずれを抑制する. 本手法は電子情報通信学会 ソサエティ大会 (2026) にて発表する. 

# 環境構築
Python 3.10.20
ROS Noetic (Install guide is here) (ROS2 imprementの追加予定)
- packages
	- numpy
	- open3d
	- rosbags
	- scikit_learn
	- scipy
	- small_gicp (https://github.com/koide3/small_gicp)

# 導入
Python環境, ROS環境の構築方法は省略.

```bash
git clone https://github.com/ngtroku/No_extrasensor_preknowledge.git
cd No_extrasensor_preknowledge
pip install -r requirements.txt
```
# 使用方法
`config.json` で以下の部分を設定
- `general_settings` 
	- `input_rosbag` : 元のrosbag (ros1 rosbag) 
	- `lidar_topic` : rosbagに含まれるLiDAR点群のrostopic (sensor_msgs/msg/PointCloud2)
	- `topic_length` : 点群topicのエンコード方式. `rostopic echo /lidar_topic --noarr` から参照. `point_step` の値
	- `lidar_only` : input_rosbagがIMUのrostopicを含まない場合はtrue. それ以外はfalse.
	- `imu_topic` : rosbagに含まれるIMUのrostopic (sensor_msgs/msg/Imu)
- generate_pointcloud
	- `spoofer_x` / `spoofer_y` : 攻撃装置を設置する位置のx, y座標
	- `horizontal_resolution` / `vertical_lines` : 使用するLiDARの水平分解能 (deg) と 垂直ライン数
- `spoofed_rosbag_params`
	- `output_rosbag` : 点群改ざんをシミュレートしたrosbagの出力先
- `mitigate_params`
	- `output_rosbag` : 本手法を適用したrosbagの出力先
	- `n_subset` : 分割する小領域の数
	- `PCA_distance_threshold` : 外れ値と見なすしきい値

```bash
cd No_extrasensor_preknowledge
sudo chmod +x ./run.sh
./run.sh
```
## Sample data
サンプルデータはNagata et al. のものを使用 ([https://drive.google.com/file/d/1p4CjWm4BZ-QjbCw1od-qyOYsck_-l5Q1/view?usp=sharing](https://drive.google.com/file/d/1p4CjWm4BZ-QjbCw1od-qyOYsck_-l5Q1/view?usp=sharing))
推奨設定
- `general_settings`
	- `lidar_topic` : /velodyne_points
	- `lidar_only` : false
	- `imu_topic` : /imu
- generate_pointcloud 
	- `spoofer_x` : 15.0
	- `spoofer_y`  : 6.15
	- `horizontal_resolution` : 0.20
	- `vertical_lines` : 32

import numpy as np
import json
from pathlib import Path
import open3d as o3d
import os

import functions.spoofing_sim as spoofing_sim
import defense_alg.registration as registration

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

def binary_to_xyz(binary):
    x = binary[:, 0:4].view(dtype=np.float32).flatten()
    y = binary[:, 4:8].view(dtype=np.float32).flatten()
    z = binary[:, 8:12].view(dtype=np.float32).flatten()
    return x, y, z

def check_spoofing_condition(odom_x, odom_y, spoofer_x, spoofer_y, distance_threshold):
    dist_spoofer_to_robot = ((odom_x - spoofer_x) ** 2 + (odom_y - spoofer_y) ** 2) ** 0.5
    return dist_spoofer_to_robot <= distance_threshold

def main():
    # Initialize
    normal_pose_x, normal_pose_y = [0.0], [0.0]
    current_pose = np.eye(4)
    normal_prev_points = None

    spoofed_pose_x, spoofed_pose_y = [0.0], [0.0]
    spoofed_current_pose = np.eye(4)
    spoofed_prev_points = None
    
    # load config 
    config_path = Path('config.json')
    if not config_path.exists():
        print(f"Error: Configuration file '{config_path}' not found.")
        return

    with open(config_path, 'r') as f:
        config = json.load(f)

    base_out_dir = Path("pcd_output")
    normal_out_dir = base_out_dir / "normal"
    spoofed_out_dir = base_out_dir / "spoofed"

    normal_out_dir.mkdir(parents=True, exist_ok=True)
    spoofed_out_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output directories ensured at: {base_out_dir.resolve()}")

    # Config
    bag_path = Path(config["general_settings"]["input_rosbag"])
    topic_name = config["general_settings"]["lidar_topic"]
    topic_length = config["general_settings"]["topic_length"]

    # spoofing simulation settings
    spoofer_x, spoofer_y = config["generate_pointcloud"]["spoofer_x"], config["generate_pointcloud"]["spoofer_y"]
    spoofing_range = config["generate_pointcloud"]["spoofing_angle_range"]

    typestore = get_typestore(Stores.ROS1_NOETIC)
    frame_idx = 0  # counter

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == topic_name]
        
        if not connections:
            print(f"Topic '{topic_name}' not found in the bag.")
            return

        print(f"Reading bag: {bag_path}")
        print(f"Target topic: {topic_name}")
        print("=" * 50)

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)

            if connection.msgtype != 'sensor_msgs/msg/PointCloud2':
                continue

            point_step = topic_length
            num_points = msg.width * msg.height
            
            if num_points == 0 or point_step == 0:
                print(f"Time: {timestamp / 1e9:.3f} | Empty PointCloud")
                continue

            # Get xyz coords from binary (raw) data.
            bin_points = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, point_step)
            x, y, z = binary_to_xyz(bin_points)

            normal_current_points = np.column_stack((x, y, z))

            # --- Normal Odomtry Calculation ---
            if normal_prev_points is None:
                normal_prev_points = normal_current_points
            else:
                result = registration.registration(source=normal_prev_points, target=normal_current_points)
                current_pose = current_pose @ np.linalg.inv(result.T_target_source)
                normal_prev_points = normal_current_points

            normal_pose_x.append(current_pose[0, 3])
            normal_pose_y.append(current_pose[1, 3])

            # -------------------------------------------------------------
            # LiDAR Spoofing Simulation
            # -------------------------------------------------------------
            spoofing_angle = spoofing_sim.decide_spoofing_param(normal_pose_x[-2], normal_pose_y[-2], spoofer_x, spoofer_y)
            is_spoofing = check_spoofing_condition(normal_pose_x[-2], normal_pose_y[-2], spoofer_x, spoofer_y, config["generate_pointcloud"]["distance_threshold"])

            if is_spoofing:
                x_rem, y_rem, z_rem, x_sp, y_sp, z_sp = spoofing_sim.spoof_main(normal_current_points, spoofing_angle, spoofing_range)
            else:
                x_rem, y_rem, z_rem = x.copy(), y.copy(), z.copy()
                x_sp, y_sp, z_sp = np.array([]), np.array([]), np.array([])

            # Adjust Pointcloud fotmat (N, 3)
            spoofed_points = np.vstack((np.concatenate((x_rem, x_sp)), 
                                        np.concatenate((y_rem, y_sp)), 
                                        np.concatenate((z_rem, z_sp)))).T

            if spoofed_prev_points is None:
                spoofed_prev_points = spoofed_points
            else:
                spoofed_result = registration.registration(source=spoofed_prev_points, target=spoofed_points)
                print(f"[Frame {frame_idx:04d}] Simulation done.")
                spoofed_current_pose = spoofed_current_pose @ np.linalg.inv(spoofed_result.T_target_source)
                spoofed_prev_points = spoofed_points

            spoofed_pose_x.append(spoofed_current_pose[0, 3])
            spoofed_pose_y.append(spoofed_current_pose[1, 3])

            # Save normal pointcloud
            pcd_normal = o3d.geometry.PointCloud()
            pcd_normal.points = o3d.utility.Vector3dVector(normal_current_points)
            o3d.io.write_point_cloud(str(normal_out_dir / f"normal_{frame_idx:04d}.pcd"), pcd_normal)

            # Save tampered pointcloud
            pcd_spoofed = o3d.geometry.PointCloud()
            pcd_spoofed.points = o3d.utility.Vector3dVector(spoofed_points)
            o3d.io.write_point_cloud(str(spoofed_out_dir / f"spoofed_{frame_idx:04d}.pcd"), pcd_spoofed)

            frame_idx += 1

    print(f"\n[Completed] {frame_idx} frames PCD pointcloud is saved at '{base_out_dir}/' .")

if __name__ == "__main__":
    main()

from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

import numpy as np
import open3d as o3d
from pathlib import Path
import json
import sys

def load_points(pcd_path):
    pcd = o3d.io.read_point_cloud(str(pcd_path))
    return np.asarray(pcd.points)

def create_pointcloud2(points, seq, stamp_ns, frame_id, typestore):
    blob = points.astype(np.float32).tobytes()
    data_array = np.frombuffer(blob, dtype=np.uint8)
    
    PointField = typestore.types['sensor_msgs/msg/PointField']
    fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
    ]
    
    Header = typestore.types['std_msgs/msg/Header']
    Timestamp = typestore.types['builtin_interfaces/msg/Time']
    ros_time = Timestamp(sec=int(stamp_ns // 1e9), nanosec=int(stamp_ns % 1e9))
    
    header_kwargs = {'stamp': ros_time, 'frame_id': frame_id}
    if hasattr(Header, '__dataclass_fields__') and 'seq' in Header.__dataclass_fields__:
        header_kwargs['seq'] = seq
    elif hasattr(Header, 'seq'):
        header_kwargs['seq'] = seq
    else:
        try:
            Header(seq=seq, stamp=ros_time, frame_id=frame_id)
            header_kwargs['seq'] = seq
        except TypeError:
            pass

    header = Header(**header_kwargs)
    PointCloud2 = typestore.types['sensor_msgs/msg/PointCloud2']
    
    return PointCloud2(
        header=header, 
        height=1, 
        width=points.shape[0], 
        fields=fields,
        is_bigendian=False, 
        point_step=12, 
        row_step=12 * points.shape[0],
        data=data_array, 
        is_dense=True
    )

def generate_main(config):
    pcd_dir = Path(config['spoofed_rosbag_params']['input_dir'])
    bag_path = Path(config['general_settings']['input_rosbag'])
    output_bag_path = Path(config['spoofed_rosbag_params']['output_rosbag'])
    lidar_topic = config['general_settings']['lidar_topic']
    imu_topic = config['general_settings']['imu_topic']

    if not pcd_dir.exists():
        print(f"Error: PCD directory '{pcd_dir}' does not exist.")
        sys.exit(1)
        
    pcd_list = sorted(list(pcd_dir.glob('*.pcd')))
    total_pcds = len(pcd_list)
    
    if total_pcds == 0:
        print(f"Error: No PCD files found in {pcd_dir}")
        sys.exit(1)

    if output_bag_path.exists():
        output_bag_path.unlink()
        print(f"Removed existing output bag: {output_bag_path}")

    output_bag_path.parent.mkdir(parents=True, exist_ok=True)

    typestore = get_typestore(Stores.ROS1_NOETIC)
    cnt = 0

    print("=" * 60)
    print(f"Input Bag  : {bag_path}")
    print(f"Output Bag : {output_bag_path}")
    print(f"Target PCDs: {total_pcds} files from '{pcd_dir}'")
    print("=" * 60)

    if config['general_settings']['lidar_only'] == True:
        
        with AnyReader([bag_path], default_typestore=typestore) as reader:
            with Writer(output_bag_path) as writer:
                
                lidar_conn_out = writer.add_connection(lidar_topic, 'sensor_msgs/msg/PointCloud2', typestore=typestore)
                imu_conn_out = writer.add_connection(imu_topic, 'sensor_msgs/msg/Imu', typestore=typestore)

                connections = [x for x in reader.connections if x.topic in [lidar_topic, imu_topic]]

                for connection, timestamp, rawdata in reader.messages(connections=connections):

                    if cnt >= total_pcds and connection.topic == lidar_topic:
                        print(f"\n[Info] All {total_pcds} PCD files have been consumed. Stopping bag generation.")
                        break
                        
                    if connection.topic == lidar_topic:
                        msg = reader.deserialize(rawdata, connection.msgtype)
                        msg_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                        
                        current_pcd_path = pcd_list[cnt]
                        written_cloud = load_points(current_pcd_path)
                        
                        out_msg = create_pointcloud2(written_cloud, cnt, msg_ns, msg.header.frame_id, typestore)
                        serialized_msg = typestore.serialize_ros1(out_msg, lidar_conn_out.msgtype)
                        
                        writer.write(lidar_conn_out, timestamp, serialized_msg)
                        
                        cnt += 1
                        if cnt % 100 == 0:
                            print(f"Progress: {cnt}/{total_pcds} frames merged...")

    else:
        with AnyReader([bag_path], default_typestore=typestore) as reader:
            with Writer(output_bag_path) as writer:
                
                lidar_conn_out = writer.add_connection(lidar_topic, 'sensor_msgs/msg/PointCloud2', typestore=typestore)
                imu_conn_out = writer.add_connection(imu_topic, 'sensor_msgs/msg/Imu', typestore=typestore)

                connections = [x for x in reader.connections if x.topic in [lidar_topic, imu_topic]]

                for connection, timestamp, rawdata in reader.messages(connections=connections):

                    if cnt >= total_pcds and connection.topic == lidar_topic:
                        print(f"\n[Info] All {total_pcds} PCD files have been consumed. Stopping bag generation.")
                        break

                    if connection.topic == imu_topic:
                        writer.write(imu_conn_out, timestamp, rawdata)
                        
                    elif connection.topic == lidar_topic:
                        msg = reader.deserialize(rawdata, connection.msgtype)
                        msg_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                        
                        current_pcd_path = pcd_list[cnt]
                        written_cloud = load_points(current_pcd_path)
                        
                        out_msg = create_pointcloud2(written_cloud, cnt, msg_ns, msg.header.frame_id, typestore)
                        serialized_msg = typestore.serialize_ros1(out_msg, lidar_conn_out.msgtype)
                        
                        writer.write(lidar_conn_out, timestamp, serialized_msg)
                        
                        cnt += 1
                        if cnt % 100 == 0:
                            print(f"Progress: {cnt}/{total_pcds} frames merged...")


    print(f"\n[Success] Finished creating spoofed rosbag!")
    print(f"Saved to: {output_bag_path.resolve()}")

if __name__ == "__main__":
    config_path = Path('config.json')
    if not config_path.exists():
        print(f"Error: Configuration file '{config_path}' not found.")
        sys.exit(1)

    with open(config_path, 'r') as f:
        config = json.load(f)

    generate_main(config)
import os

import builtin_interfaces.msg
import numpy as np
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import RadarPointCloud
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


class LogConverter:
    def __init__(self, data_set_name, data_set_path, output_path):
        self._data_set_name = data_set_name
        self._data_set_path = data_set_path
        self._output_path = output_path

        if os.path.exists(self._data_set_path):
            self._nusc = NuScenes(
                version=self._data_set_name,
                dataroot=self._data_set_path,
                verbose=True,
            )
        else:
            raise FileNotFoundError(
                f"Path {self._data_set_path} does not exist"
            )

        if self._output_path and not os.path.exists(
            self._output_path
        ):
            os.makedirs(self._output_path)

    def convert_all_scenes(self):
        for scene in self._nusc.scene:
            print(f"Processing scene: {scene['name']}")
            self.convert_scene(scene)

    def convert_scene(self, scene):
        mcap_file_path = os.path.join(
            self._output_path, f"{scene['name']}.mcap"
        )

        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(
                uri=mcap_file_path, storage_id="mcap"
            ),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )

        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/LIDAR_TOP",
                type="sensor_msgs/msg/PointCloud2",
                serialization_format="cdr",
            )
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/EGO_POSE",
                type="nav_msgs/msg/Odometry",
                serialization_format="cdr",
            )
        )

        # Radar sensor setup (5 radar topics)
        radar_sensors = [
            "RADAR_FRONT",
            "RADAR_FRONT_LEFT",
            "RADAR_FRONT_RIGHT",
            "RADAR_BACK_LEFT",
            "RADAR_BACK_RIGHT",
        ]

        for radar in radar_sensors:
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=f"/{radar}",
                    type="sensor_msgs/msg/PointCloud2",
                    serialization_format="cdr",
                )
            )

        # TF topics for static and dynamic transforms
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/tf_static",
                type="tf2_msgs/msg/TFMessage",
                serialization_format="cdr",
            )
        )

        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/tf",
                type="tf2_msgs/msg/TFMessage",
                serialization_format="cdr",
            )
        )

        sample_token = scene["first_sample_token"]
        sample = self._nusc.get("sample", sample_token)

        # Set static transforms at the beginning of the scene (using timestamp of the first sample)
        first_timestamp = sample["timestamp"] * 1000
        self._write_static_transforms(writer, sample, first_timestamp)

        while sample_token:
            sample = self._nusc.get("sample", sample_token)

            for sensor_name, data_token in sample["data"].items():
                sample_data = self._nusc.get(
                    "sample_data", data_token
                )

                # Conver timestamp to nanoseconds for ROS (NuScenes uses microseconds)
                timestamp_ns = sample_data["timestamp"] * 1000

                # 1. LIDAR
                if "LIDAR" in sensor_name:
                    self._write_lidar(
                        writer, sensor_name, sample_data, timestamp_ns
                    )

                # 2. RADAR topics
                if "RADAR" in sensor_name:
                    self._write_radar(
                        writer, sensor_name, sample_data, timestamp_ns
                    )

                # 3. EGO_POSE (odometry and TF)
                self._write_ego_pose(
                    writer, sample_data, timestamp_ns
                )

            sample_token = sample["next"]

        del writer
        print(f"Finished: {mcap_file_path}")

    def _get_ros_time(self, ts_ns):
        t = builtin_interfaces.msg.Time()
        t.sec = ts_ns // 1_000_000_000
        t.nanosec = ts_ns % 1_000_000_000
        return t

    def _write_lidar(
        self, writer, sensor_name, sample_data, timestamp_ns
    ):
        file_path = os.path.join(
            self._nusc.dataroot, sample_data["filename"]
        )
        # NuScenes przechowuje LIDAR jako float32 (x, y, z, intensity, ring_index)
        points = np.fromfile(file_path, dtype=np.float32).reshape(
            (-1, 5)
        )

        msg = PointCloud2()
        msg.header = Header(
            frame_id=sensor_name,
            stamp=self._get_ros_time(timestamp_ns),
        )
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.point_step = 16  # x, y, z, intensity (4 * 4 bytes)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        # Definicja pól (tylko x, y, z, intensity dla prostoty)
        msg.fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="intensity",
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        msg.data = points[:, :4].tobytes()

        writer.write(
            f"/{sensor_name}",
            serialize_message(msg),
            timestamp_ns,
        )

    def _write_radar(
        self, writer, sensor_name, sample_data, timestamp_ns
    ):
        file_path = os.path.join(
            self._nusc.dataroot, sample_data["filename"]
        )

        # Use RadarPointCloud helper to read radar data (returns 18 features)
        pcl = RadarPointCloud.from_file(file_path)
        points = pcl.points.T  # Transpose (N, 18)

        msg = PointCloud2()
        msg.header = Header(
            frame_id=sensor_name,
            stamp=self._get_ros_time(timestamp_ns),
        )
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False

        # Select fields: 0:x, 1:y, 2:z, 5:rcs, 8:vx_comp, 9:vy_comp
        # Each point will have 6 fields, each 4 bytes (float32) => point_step = 24
        msg.point_step = 24
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        msg.fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="rcs",
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="vx",
                offset=16,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="vy",
                offset=20,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]

        # Set radar data (x, y, z, rcs, vx_comp, vy_comp)
        # RadarPointCloud.points: 0:x, 1:y, 2:z, 5:rcs, 8:vx_comp, 9:vy_comp
        selected_indices = [0, 1, 2, 5, 8, 9]
        radar_data = points[:, selected_indices].astype(np.float32)
        msg.data = radar_data.tobytes()

        writer.write(
            f"/{sensor_name}",
            serialize_message(msg),
            timestamp_ns,
        )

    def _write_ego_pose(self, writer, sample_data, timestamp_ns):
        ego_pose = self._nusc.get(
            "ego_pose", sample_data["ego_pose_token"]
        )
        ros_time = self._get_ros_time(timestamp_ns)

        # Set Odometry message
        odo_msg = Odometry()
        odo_msg.header = Header(frame_id="map", stamp=ros_time)
        odo_msg.child_frame_id = "base_link"

        odo_msg.pose.pose.position.x = ego_pose["translation"][0]
        odo_msg.pose.pose.position.y = ego_pose["translation"][1]
        odo_msg.pose.pose.position.z = ego_pose["translation"][2]
        odo_msg.pose.pose.orientation.w = ego_pose["rotation"][0]
        odo_msg.pose.pose.orientation.x = ego_pose["rotation"][1]
        odo_msg.pose.pose.orientation.y = ego_pose["rotation"][2]
        odo_msg.pose.pose.orientation.z = ego_pose["rotation"][3]

        writer.write(
            "/EGO_POSE", serialize_message(odo_msg), timestamp_ns
        )

        # Set TF message (Dynamic transform)
        tf_msg = TFMessage()
        t = TransformStamped()
        t.header.stamp = ros_time
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        # Same translation and rotation as in Odometry
        t.transform.translation.x = ego_pose["translation"][0]
        t.transform.translation.y = ego_pose["translation"][1]
        t.transform.translation.z = ego_pose["translation"][2]
        t.transform.rotation.w = ego_pose["rotation"][0]
        t.transform.rotation.x = ego_pose["rotation"][1]
        t.transform.rotation.y = ego_pose["rotation"][2]
        t.transform.rotation.z = ego_pose["rotation"][3]

        tf_msg.transforms.append(t)

        writer.write("/tf", serialize_message(tf_msg), timestamp_ns)

    def _write_static_transforms(self, writer, sample, timestamp_ns):
        tf_msg = TFMessage()

        for sensor_name, data_token in sample["data"].items():
            sample_data = self._nusc.get("sample_data", data_token)
            calibrated_sensor = self._nusc.get(
                "calibrated_sensor",
                sample_data["calibrated_sensor_token"],
            )

            t = TransformStamped()
            t.header.stamp = self._get_ros_time(timestamp_ns)
            t.header.frame_id = "base_link"
            t.child_frame_id = sensor_name

            # Position
            t.transform.translation.x = calibrated_sensor[
                "translation"
            ][0]
            t.transform.translation.y = calibrated_sensor[
                "translation"
            ][1]
            t.transform.translation.z = calibrated_sensor[
                "translation"
            ][2]

            # Orientation (quaternion)
            t.transform.rotation.w = calibrated_sensor["rotation"][0]
            t.transform.rotation.x = calibrated_sensor["rotation"][1]
            t.transform.rotation.y = calibrated_sensor["rotation"][2]
            t.transform.rotation.z = calibrated_sensor["rotation"][3]

            tf_msg.transforms.append(t)

        writer.write(
            "/tf_static",
            serialize_message(tf_msg),
            timestamp_ns,
        )


if __name__ == "__main__":
    # Set correct paths
    lc = LogConverter(
        "v1.0-mini",
        "/home/maciej/Downloads/v1.0-mini",
        "/home/maciej/Downloads/output_mcap",
    )
    lc.convert_all_scenes()

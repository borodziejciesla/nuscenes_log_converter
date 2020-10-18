import sys
import abc

sys.path.insert(1, '/home/maciek/Documents/git/ros_tutorial/mht_ws/src/log_converter/scripts/nuscenes_log_converter')
import sensors_data_pb2
import sys

class SceneWriterBase:
    def __init__(self, sensor):
        self._file_path = None
        self._is_open = False
        self._file = None
        self._sensor = sensor

    @abc.abstractmethod
    def createFile(self, file_path):
        pass
    
    @abc.abstractmethod
    def writeScan(self, scan_data):
        pass
    
    @abc.abstractmethod
    def closeFile(self):
        self.__file = ''
        self._is_open = False
        self._file = None

    def isOpen(self):
        if self._file is None:
            return False
        else:
            return not self._file.closed


class ProtobufSceneWriter(SceneWriterBase):
    def __init__(self, sensor):
        super().__init__(sensor)

        self.__sensor_write_function = None
        self.__log_data = None

        if self._sensor == 'lidar':
            self.__sensor_write_function = self.__lidarWriter
            self.__log_data = sensors_data_pb2.LidarLogData()
        elif self._sensor == 'radar':
            self.__sensor_write_function = self.__radarWriter
            self.__log_data = sensors_data_pb2.RadarLogData()
        elif self._sensor == 'camera':
            self.__sensor_write_function = self.__cameraWriter
            self.__log_data = sensors_data_pb2.ObjectsLogData()
        elif self._sensor == 'host':
            self.__sensor_write_function = self.__hostWriter
            self.__log_data = sensors_data_pb2.HostLogData()
        else:
            print('Invalid sensor type')

    def createFile(self, file_path):
        self._file_path = file_path
        self._file = open(file_path, "wb")

        if self._file:
            self._is_open = True
        else:
            self._is_open = False

    def writeScan(self, scan_data):
        self.__sensor_write_function(scan_data)

    def closeFile(self):
        if self.isOpen():
            if self.__log_data is not None:
                self._file.write(self.__log_data.SerializeToString())
            self._file.close()
        else:
            print('File {} is not open!'.format(self._file_path))
        super().closeFile()

    def __radarWriter(self, scan_data):
        scan = self.__log_data.scan_data.add()

        # Timestamp
        scan.time_stamp = scan_data['timestamp']

        # Origin
        scan.sensor_origin.x = scan_data['sensor_calibrations'].x
        scan.sensor_origin.y = scan_data['sensor_calibrations'].y
        scan.sensor_origin.z = scan_data['sensor_calibrations'].z
        scan.sensor_origin.yaw = scan_data['sensor_calibrations'].yaw
        scan.sensor_origin.pitch = scan_data['sensor_calibrations'].pitch
        scan.sensor_origin.roll = scan_data['sensor_calibrations'].roll

        # Detections
        for detection in scan_data['point_cloud']:
            output_detection = scan.detections.add()
            output_detection.id = detection.id
            output_detection.x = detection.x
            output_detection.y = detection.y
            output_detection.z = detection.z
            output_detection.range = detection.rng
            output_detection.azimuth = detection.azimuth
            output_detection.elevation = detection.elevation
            output_detection.range_rate = detection.range_rate
            if detection.classification == 'MOVING':
                output_detection.classification = sensors_data_pb2.RadarDetection.RADAR_CASS_DYNAMIC
            elif detection.classification == 'STATIC':
                output_detection.classification = sensors_data_pb2.RadarDetection.RADAR_CASS_STATIC
            else:
                output_detection.classification = sensors_data_pb2.RadarDetection.RADAR_CASS_UNKNOWN

    def __lidarWriter(self, scan_data):
        scan = self.__log_data.scan_data.add()

        # Timestamp
        scan.time_stamp = scan_data['timestamp']

        # Origin
        scan.sensor_origin.x = scan_data['sensor_calibrations'].x
        scan.sensor_origin.y = scan_data['sensor_calibrations'].y
        scan.sensor_origin.z = scan_data['sensor_calibrations'].z
        scan.sensor_origin.yaw = scan_data['sensor_calibrations'].yaw
        scan.sensor_origin.pitch = scan_data['sensor_calibrations'].pitch
        scan.sensor_origin.roll = scan_data['sensor_calibrations'].roll

        # Detections
        for detection in scan_data['point_cloud']:
            output_detection = scan.detections.add()
            output_detection.id = detection.id
            output_detection.x = detection.x
            output_detection.y = detection.y
            output_detection.z = detection.z
            output_detection.range = detection.rng
            output_detection.azimuth = detection.azimuth
            output_detection.elevation = detection.elevation
            output_detection.classification = sensors_data_pb2.LidarDetecion.LIDAR_CLASS_UNKNOWN

    def __cameraWriter(self, scan_data):
        scan = self.__log_data.scan_data.add()

        # Timestamp
        scan.time_stamp = scan_data['timestamp']

        # Origin
        scan.sensor_origin.x = scan_data['sensor_calibrations'].x
        scan.sensor_origin.y = scan_data['sensor_calibrations'].y
        scan.sensor_origin.z = scan_data['sensor_calibrations'].z
        scan.sensor_origin.yaw = scan_data['sensor_calibrations'].yaw
        scan.sensor_origin.pitch = scan_data['sensor_calibrations'].pitch
        scan.sensor_origin.roll = scan_data['sensor_calibrations'].roll

        # Objects
        for obj in scan_data['objects']:
            output_object = scan.detections.add()
            output_object.id = obj.id
            output_object.x = obj.x
            output_object.x_std = obj.x_std
            output_object.y = obj.y
            output_object.y_std = obj.y_std
            output_object.vx = obj.v_x
            output_object.vx_std = obj.v_x_std
            output_object.vy = obj.v_y
            output_object.vy_std = obj.v_y_std
            output_object.width = obj.width
            output_object.width_std = 0.0
            output_object.length = obj.length
            output_object.length_std = 0.0
            output_object.movement_classification = sensors_data_pb2.Object.OBJECT_MOVEMENT_CLASS_UNKNOWN
            output_object.object_class = sensors_data_pb2.Object.OBJECT_CLASS_UNKNOWN

    def __hostWriter(self, scan_data):
        scan = self.__log_data.scan_data.add()

        # Timestamp
        scan.time_stamp = scan_data.time_stamp

        # Kinematic data
        scan.x = scan_data.x
        scan.v_x = scan_data.v_x
        scan.y = scan_data.y
        scan.v_y = scan_data.v_y
        scan.z = scan_data.z
        scan.v_z = scan_data.v_z

        scan.yaw = scan_data.yaw
        scan.yaw_rate = scan_data.yaw_rate
        scan.pitch = scan_data.pitch
        scan.pitch_rate = scan_data.pitch_rate
        scan.roll = scan_data.roll
        scan.roll_rate = scan_data.roll_rate

    def __sensrCalibrationWriter(self, scan_data):
        pass

class MatlabSceneWriter(SceneWriterBase):
    def __init__(self, sensor):
        super().__init__(sensor)
import abc
from nuscenes.utils.data_classes import RadarPointCloud, LidarPointCloud
from dataclasses import dataclass
from math import sqrt, atan2, hypot
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation

@dataclass()
class Origin:
    x: float
    y: float
    z: float

    yaw: float
    pitch: float
    roll: float

@dataclass()
class HostData:
    time_stamp: float

    x: float
    v_x: float
    y: float
    v_y: float
    z: float
    v_z: float

    yaw: float
    yaw_rate: float
    pitch: float
    pitch_rate: float
    roll: float
    roll_rate: float

@dataclass()
class RadarDetection:
    id: int

    x: float
    x_std: float
    y: float
    y_std: float
    z: float
    z_std: float

    rng: float
    rng_std: float
    azimuth: float
    azimuth_std: float
    elevation: float
    elevation_std: float

    range_rate: float
    range_rate_std: float

    classification: str

@dataclass()
class LidarDetection:
    id: int

    x: float
    x_std: float
    y: float
    y_std: float
    z: float
    z_std: float

    rng: float
    rng_std: float
    azimuth: float
    azimuth_std: float
    elevation: float
    elevation_std: float

@dataclass()
class Object:
    id: int

    x: float
    x_std: float
    y: float
    y_std: float
    z: float
    z_std: float

    v_x: float
    v_x_std: float
    v_y: float
    v_y_std: float
    v_z: float
    v_z_std: float

    yaw: float
    yaw_std: float
    pitch: float
    pitch_std: float
    roll: float
    roll_std: float

    yaw_rate: float
    yaw_rate_std: float
    pitch_rate: float
    pitch_rate_std: float
    roll_rate: float
    roll_rate_std: float

    length: float
    width: float
    height: float

    label: str


class BaseSensorScanReader:
    def __init__(self):
        pass

    @abc.abstractmethod
    def readScan(self, data_file_path, timestamp, calibrated_sensor=None, boxes=None):
        self._data = {}
        if not (calibrated_sensor == None):
            self._data['sensor_calibrations'] = self.readSensorCalibrations(calibrated_sensor)
        self._data['timestamp'] = timestamp

    def readSensorCalibrations(self, calibrated_sensor):
        x = calibrated_sensor['translation'][0]
        y = calibrated_sensor['translation'][1]
        z = calibrated_sensor['translation'][2]

        yaw, pitch, roll = Quaternion2YPR(calibrated_sensor['rotation'])
        
        return Origin(x, y, z, yaw, pitch, roll)


class DetectionReader(BaseSensorScanReader):
    @abc.abstractmethod
    def readScan(self, data_file_path, timestamp, calibrated_sensor=None, boxes=None):
        super().readScan(data_file_path, timestamp, calibrated_sensor)

    def _convertDetections(self, point_cloud):
        self._points_array = point_cloud.points
        points_list = []
        for i in range(1, self._points_array.shape[1]):
            points_list.append(self._createDetection(i))
        return points_list

    @abc.abstractmethod
    def _createDetection(self, det_idx):
        pass


class RadarScanReader(DetectionReader):
    def __init__(self):
        pass

    def readScan(self, data_file_path, timestamp, calibrated_sensor=None, boxes=None):
        super().readScan(data_file_path, timestamp, calibrated_sensor)
        self._data['point_cloud'] = self._convertDetections(RadarPointCloud.from_file(data_file_path))
        return self._data

    def _createDetection(self, det_idx):
        x = self._points_array[0][det_idx]
        y = self._points_array[1][det_idx]
        z = self._points_array[2][det_idx]

        rng = sqrt(x*x + y*y + z*z)
        azimuth = atan2(y, x)
        elevation = atan2(z, hypot(x, y))

        vx = self._points_array[6][det_idx]
        vy = self._points_array[7][det_idx]
        range_rate = hypot(vx, vy)

        classification = 'UNKNOWN'
        if range_rate > 1.0:
            classification = 'MOVING'
        else:
            classification = 'STATIC'

        return RadarDetection(det_idx, x, 0, y, 0, z, 0, rng, 0, azimuth, 0, elevation, 0, range_rate, 0, classification)


class LidarScanReader(DetectionReader):
    def __init__(self):
        pass

    def readScan(self, data_file_path, timestamp, calibrated_sensor=None, boxes=None):
        super().readScan(data_file_path, timestamp, calibrated_sensor)
        self._data['point_cloud'] = self._convertDetections(LidarPointCloud.from_file(data_file_path))
        return self._data

    def _createDetection(self, det_idx):
        x = self._points_array[0][det_idx]
        y = self._points_array[1][det_idx]
        z = self._points_array[2][det_idx]

        rng = sqrt(x*x + y*y + z*z)
        azimuth = atan2(y, x)
        elevation = atan2(z, hypot(x, y))

        return LidarDetection(det_idx, x, 0, y, 0, z, 0, rng, 0, azimuth, 0, elevation, 0)


class VisionScanReader(BaseSensorScanReader):
    def __init__(self):
        pass

    def readScan(self, data_file_path, timestamp, calibrated_sensor=None, boxes=None):
        super().readScan(data_file_path, timestamp, calibrated_sensor)
        self._data['objects'] = self.__convertObjects(boxes)
        return self._data

    def __convertObjects(self, boxes):
        self.__objects_array = boxes
        objects_list = []
        for i in range(1, len(boxes)):
            objects_list.append(self.__createObject(i))
        return objects_list

    def __createObject(self, object_idx):
        object_raw = self.__objects_array[object_idx]

        x = object_raw.center[0]
        x_std = 0
        y = object_raw.center[1]
        y_std = 0
        z = object_raw.center[2]
        z_std = 0

        v_x = 0
        v_x_std = 0
        v_y = 0
        v_y_std = 0
        v_z = 0
        v_z_std = 0

        yaw, pitch, roll = Quaternion2YPR(object_raw.orientation)
        yaw_std = 0
        pitch_std = 0
        roll_std = 0

        yaw_rate = 0
        yaw_rate_std = 0
        pitch_rate = 0
        pitch_rate_std = 0
        roll_rate = 0
        roll_rate_std = 0

        length = object_raw.wlh[0]
        width = object_raw.wlh[0]
        height = object_raw.wlh[0]

        label = object_raw.name

        return Object(object_idx, x, x_std, y, y_std, z, z_std, 
            v_x, v_x_std, v_y, v_y_std, v_z, v_z_std, 
            yaw, yaw_std, pitch, pitch_std, roll, roll_std, 
            yaw_rate, yaw_rate_std, pitch_rate, pitch_rate_std, roll_rate, roll_rate_std, 
            length, width, height, 
            label)

class HostDataReader:
    def __init(self):
        pass

    def readHostData(self, raw_ego_data):
        time_stamp = raw_ego_data['timestamp']

        x = raw_ego_data['translation'][0]
        y = raw_ego_data['translation'][1]
        z = raw_ego_data['translation'][2]

        yaw, pitch, roll = Quaternion2YPR(raw_ego_data['rotation'])

        return HostData(time_stamp, x, 0.0, y, 0.0, z, 0.0, yaw, 0.0, pitch, 0.0, roll, 0.0)

def Quaternion2YPR(q):
    rot = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    euler = rot.as_euler('xyz', degrees=False)
    return euler[2], euler[1], euler[0]
import os
import scan_readers
import writers
from nuscenes.nuscenes import NuScenes

class LogConverter:
    __radar_reader = scan_readers.RadarScanReader()
    __lidar_reader = scan_readers.LidarScanReader()
    __vision_reader = scan_readers.VisionScanReader()
    __host_data_reader = scan_readers.HostDataReader()

    def __init__(self, data_set_name, data_set_path, output_path=''):
        self.__scan_data = {}
        self.__writers = {}

        if os.path.exists(data_set_path):
            self.__nusc = NuScenes(version=data_set_name, dataroot=data_set_path, verbose=True)
        else:
            print('Given path: {}, does not exist'.format(data_set_path))

        self.__output_path = output_path
        if not (self.__output_path == ''):
            if not os.path.exists(self.__output_path):
                os.makedirs(self.__output_path)

    def convertAllScenes(self):
        for scene in self.__nusc.scene:
            self.__scene_name = scene['name']
            self.convertScene(scene)

    def convertScene(self, scene):
        first_sample_token = scene['first_sample_token']
        sample = self.__nusc.get('sample', first_sample_token)
        while sample['next']:
            sample = self.__nusc.get('sample', sample['next'])
            for sensor in sample['data']:
                token = sample['data'][sensor]
                self.__sensor_data = self.__nusc.get('sample_data', token)
                self.readHostData()
                self.readSensorSampleData(sensor, token)
            self.__writeScan()
        self.__writeScene()

    def readHostData(self):
        ego_pose_raw = self.__nusc.get('ego_pose', self.__sensor_data['ego_pose_token'])
        self.__scan_data['HOST_DATA'] = self.__host_data_reader.readHostData(ego_pose_raw)

    def readSensorSampleData(self, sensor, token):
        data_file_path = self.__nusc.dataroot + '/' + self.__sensor_data['filename']
        calibrated_sensor_raw = self.__nusc.get('calibrated_sensor', self.__sensor_data['calibrated_sensor_token'])
        ts = self.__sensor_data['timestamp']

        if 'RADAR' in sensor:
            self.__scan_data[sensor] = self.__radar_reader.readScan(data_file_path, ts, calibrated_sensor_raw)
        elif 'LIDAR' in sensor:
            self.__scan_data[sensor] = self.__lidar_reader.readScan(data_file_path, ts, calibrated_sensor_raw)
        elif 'CAM' in sensor:
            _, boxes, _ = self.__nusc.get_sample_data(token)
            self.__scan_data[sensor] = self.__vision_reader.readScan(data_file_path, ts, calibrated_sensor_raw, boxes)
        else:
            print('Sensor unknown')
            self.__scan_data = None
    
    def __writeScan(self):
        for sensor_name, sensor_data in self.__scan_data.items():
            if not (sensor_name in self.__writers):
                self.__writers[sensor_name] = self.__createWriter(sensor_name)
            
            self.__writers[sensor_name].writeScan(sensor_data)

    def __createWriter(self, sensor_key):
        writer = None
        ext = None

        if 'RADAR' in sensor_key:
            writer =  writers.ProtobufSceneWriter('radar')
            ext = '.rad'
        elif 'LIDAR' in sensor_key:
            writer =  writers.ProtobufSceneWriter('lidar')
            ext = '.lid'
        elif 'CAM' in sensor_key:
            writer =  writers.ProtobufSceneWriter('camera')
            ext = '.cam'
        elif 'HOST' in sensor_key:
            writer =  writers.ProtobufSceneWriter('host')
            ext = '.host'
        else:
            writer = None

        if writer:
            file_name = self.__scene_name + '_' + sensor_key + ext
            writer.createFile(self.__createFilePath(file_name))
        return writer

    def __createFilePath(self, file_name):
        if self.__output_path == '':
            return file_name
        else:
            return self.__output_path + '/' + file_name

    def __writeScene(self):
        for writer in self.__writers:
            self.__writers[writer].closeFile()
        self.__writers = {}
    
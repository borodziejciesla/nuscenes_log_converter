from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from log_reader.log_reader import LogReader
import os
import numpy as np
import time
import scipy.linalg

class Ploter:
    def __init__(self, detections_logs_list=[], objects_log_list=[], host_data_log=''):
        # Copy logs paths
        self.__detections_logs_list = detections_logs_list
        self.__objects_log_list = objects_log_list
        self.__host_data_log = host_data_log

        # Open Readers
        self.__detections_readers = []
        for detections_log in self.__detections_logs_list:
            self.__detections_readers.append(LogReader(detections_log))
        
        self.__objects_readers = []
        for objects_log in self.__objects_log_list:
            self.__objects_readers.append(LogReader(objects_log))
        
        if self.__host_data_log:
            self.__host_data_reader = LogReader(self.__host_data_log)

    def PlotDetections(self, scan_data):
        yaw_rad = np.deg2rad(scan_data.sensor_origin.yaw)
        pitch_rad = np.deg2rad(scan_data.sensor_origin.pitch)
        roll_rad = np.deg2rad(scan_data.sensor_origin.roll)

        cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
        cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
        cr, sr = np.cos(roll_rad), np.sin(roll_rad)

        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        origin_offset = np.array([scan_data.sensor_origin.x,  scan_data.sensor_origin.y, scan_data.sensor_origin.z])

        detections_number = len(scan_data.detections)
        
        distance = np.zeros((detections_number, 1))
        azimuth = np.zeros((detections_number, 1))
        range_rate = np.zeros((detections_number, 1))

        x = np.zeros((detections_number, 1))
        y = np.zeros((detections_number, 1))
        z = np.zeros((detections_number, 1))

        i = 0

        for detection in scan_data.detections:
            X = np.array([detection.x, detection.y, detection.z])
            x_rot = Rx @ Ry @ Rz @ X + origin_offset
            x[i] = x_rot[0]
            y[i] = x_rot[1]
            z[i] = x_rot[2]

            distance[i] = detection.range
            azimuth[i] = detection.azimuth
            range_rate[i] = detection.range_rate

            i += 1

        plt.plot(x, y, 'r.')
        plt.axis('equal')
        plt.grid(True)
        plt.show()
        time.sleep(0.2)
        plt.close()

    def PlotObject(self, scan_data):
        pass

    def PlotHostData(self, scan_data):
        pass


def FindVelocity(range_rate, azimuth):
    l = len(range_rate)

    H = np.zeros((l, 2))
    Y = np.zeros((l, 1))

    for idx in range(l):
        H[idx, 0] = np.cos(azimuth[idx])
        H[idx, 1] = np.sin(azimuth[idx])

        Y[idx] = -range_rate[idx]

    v = scipy.linalg.pinv(H.transpose() @ H) @ H.transpose() @ Y
    return v
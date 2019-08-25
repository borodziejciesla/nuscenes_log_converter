import sys
sys.path.append('d:\\GIT\\SLAM\\nuscenes_converter')
import os
import numpy as np
import sensors_data_pb2

class LogReader:
    def __init__(self, file_path):
        self.OpenFile(file_path)

    def OpenFile(self, file_path):
        self.__file_path = file_path
        self.__SelectFile()

        self.__file = open(self.__file_path, "rb")
        self.__sensor_data.ParseFromString(self.__file.read())
        self.__is_file_open= True
        self.__GetTimeStamps()

    def CloseFile(self):
        self.__file.close()

    def IsFileOpen(self):
        return self.__is_file_open

    def GetScansNumber(self):
        return len(self.__sensor_data.scan_data)

    def GetScanByIndex(self, scan_index):
        return self.__sensor_data.scan_data[scan_index]

    def GetScanByTime(self, selected_time_stamp, condition = lambda lhs, rhs: lhs >= rhs):
        selected_index = self.__FindIndexByTimeCondition(selected_time_stamp, condition)
        return self.GetScanByIndex(selected_index)

    def GetScansInRange(self, begin_time_stamp, end_time_stamp):
        first_index = self.__FindIndexByTimeCondition(begin_time_stamp, lambda lhs, rhs: lhs >= rhs)
        end_index = self.__FindIndexByTimeCondition(end_time_stamp, lambda lhs, rhs: lhs >= rhs)

        scans = []
        for index in range(first_index, end_index):
            scans.append(self.GetScanByIndex(index))
        return scans

    def __SelectFile(self):
        if not (self.__file_path == None):
            file_ext = os.path.splitext(self.__file_path)
            if file_ext[1] == ".rad":
                self.__sensor_data = sensors_data_pb2.RadarLogData()
            elif file_ext[1] == ".lid":
                self.__sensor_data = sensors_data_pb2.LidarLogData()
            elif file_ext[1] == ".cam":
                self.__sensor_data = sensors_data_pb2.ObjectsLogData()
            elif file_ext[1] == ".host":
                self.__sensor_data = sensors_data_pb2.HostLogData()
            else:
                raise Exception("Incorrect file extension")    
        else:
            raise Exception("File not exist")

    def __GetTimeStamps(self):
        self.__timestamps = []
        for index in range(0, self.GetScansNumber() - 1):
            scan_data = self.GetScanByIndex(index)
            self.__timestamps.append(scan_data.time_stamp)

    def __FindIndexByTimeCondition(self, selected_time_stamp, condition):
        selected_index = 0
        for ts in self.__timestamps:
            if condition(ts, selected_time_stamp):
                break
            else:
               selected_index = selected_index + 1
        print(len(self.__timestamps))
        return min(selected_index, len(self.__timestamps) - 1)

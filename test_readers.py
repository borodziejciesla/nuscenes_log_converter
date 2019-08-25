from log_reader.log_reader import LogReader
import numpy as np
from log_reader.ploters import Ploter

# Radar Data
reader = LogReader("D:\\GIT\\SLAM\\data_set\\scene-0061_RADAR_BACK_LEFT.rad")
ploter = Ploter(['D:\\GIT\\SLAM\\data_set\\scene-0061_RADAR_BACK_LEFT.rad'])

timestamps = []

if reader.IsFileOpen():
    scans_number = reader.GetScansNumber()
    for index in range(0, scans_number):
        scan_data = reader.GetScanByIndex(index)
        print("\tts = {0:.2f}".format(scan_data.time_stamp))
        timestamps.append(scan_data.time_stamp)
        ploter.PlotDetections(scan_data)

    print("Get By Timestamp")
    scan_data = reader.GetScanByTime(1533151621041512)
    print("\tts = {0:.2f}".format(scan_data.time_stamp))

    print("Get From Range")
    scans_data = reader.GetScansInRange(1533151621041512, 2 * 519855.0 + 1533151621041512)
    for scan_data in scans_data:
        print("\tts = {0:.2f}".format(scan_data.time_stamp))

reader.CloseFile()

# Host Data
reader = LogReader("D:\\GIT\\SLAM\\data_set\\scene-0103_HOST_DATA.host")

timestamps = []

if reader.IsFileOpen():
    scans_number = reader.GetScansNumber()
    for index in range(0, scans_number):
        scan_data = reader.GetScanByIndex(index)
        print("\tts = {0:.2f}".format(scan_data.time_stamp))
        timestamps.append(scan_data.time_stamp)

    print("Get By Timestamp")
    scan_data = reader.GetScanByTime(1533151621041512)
    print("\tts = {0:.2f}".format(scan_data.time_stamp))

    print("Get From Range")
    scans_data = reader.GetScansInRange(1533151621041512, 2 * 519855.0 + 1533151621041512)
    for scan_data in scans_data:
        print("\tts = {0:.2f}".format(scan_data.time_stamp))

reader.CloseFile()
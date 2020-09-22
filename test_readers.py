from log_reader.log_reader import LogReader
import numpy as np
from log_reader.ploters import Ploter
import matplotlib.pyplot as plt

# Radar Data
reader = LogReader("/home/maciek/Downloads/output/scene-0061_RADAR_FRONT.rad")
ploter = Ploter(['/home/maciek/Downloads/output/scene-0061_RADAR_FRONT.rad'])

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
reader = LogReader("/home/maciek/Downloads/output/scene-0103_HOST_DATA.host")

timestamps = []

if reader.IsFileOpen():
    scans_number = reader.GetScansNumber()

    x = np.zeros((scans_number, 1))
    y = np.zeros((scans_number, 1))
    z = np.zeros((scans_number, 1))
    yaw = np.zeros((scans_number, 1))
    pitch = np.zeros((scans_number, 1))
    roll = np.zeros((scans_number, 1))

    for index in range(0, scans_number):
        scan_data = reader.GetScanByIndex(index)
        x[index] = scan_data.x
        y[index] = scan_data.y
        z[index] = scan_data.z
        yaw[index] = scan_data.yaw
        pitch[index] = scan_data.pitch
        roll[index] = scan_data.roll
        print("\tts = {0:.2f}".format(scan_data.time_stamp))
        timestamps.append(scan_data.time_stamp)

    plt.plot(pitch)
    #plt.axis('equal')
    plt.grid(True)
    plt.show()

    print("Get By Timestamp")
    scan_data = reader.GetScanByTime(1533151621041512)
    print("\tts = {0:.2f}".format(scan_data.time_stamp))

    print("Get From Range")
    scans_data = reader.GetScansInRange(1533151621041512, 2 * 519855.0 + 1533151621041512)
    for scan_data in scans_data:
        print("\tts = {0:.2f}".format(scan_data.time_stamp))

reader.CloseFile()

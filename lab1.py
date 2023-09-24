# This is a sample Python script.
import io

import serial
import pyqtgraph as pg
import array
import numpy as np
import math

# Complementary filter constants
alpha = 0.98  # Weight for gyroscope data
dt = 0.01     # Sample interval (adjust as needed)

# Initialize initial angles
global roll, pitch, yaw
roll = 0.0
pitch = 0.0
yaw = 0.0


def complementary_filter(roll, pitch, yaw, gyro_x, gyro_y, gyro_z, x, y, z):
    # Compute roll and pitch angles using complementary filter
    roll += gyro_x * dt * math.pi/ 180
    pitch += gyro_y * dt * math.pi/ 180
    yaw += gyro_z * dt * math.pi/ 180
    # Accelerometer-based roll and pitch
    if z != 0:
        accel_pitch = math.atan(x/z)
    else: accel_pitch = math.copysign(1, x) * math.pi/2
    if z != 0:
        accel_roll = math.atan(y/z)
    else: accel_roll = math.copysign(1, y) * math.pi/2
    if y != 0:
        accel_yaw = math.atan(x/y)
    else: accel_yaw = math.copysign(1, x) * math.pi / 2
    # Combine gyro and accelerometer angles using complementary filter
    roll = alpha * roll + (1 - alpha) * accel_roll
    pitch = alpha * pitch + (1 - alpha) * accel_pitch
    yaw = alpha * yaw + (1 - alpha) * accel_yaw
    return roll, pitch, yaw

Data = serial.Serial()
Data.baudrate = 115200
Data.port = 'COM5'
Data.timeout = 0.1
Data.open()
app = pg.mkQApp()
win = pg. GraphicsLayoutWidget()
win.setWindowTitle('demo')
win.resize(1600,900)
xLength = 300
gyro_pitch = 0
gyro_roll = 0
gyro_yaw = 0

# sio = io.TextIOWrapper(io.BufferedRWPair(Data, Data))
def canFloat(data):
    try:
        float(data)
        return True
    except:
        return False

def dataProcess(data):
    data = str(data)
    dataSet = []
    datapoint = ''
    for i in data:
        if i.isdigit() or i == '-':
            datapoint += i
        elif i == ",":
            datapoint_int = int(datapoint)
            dataSet.append(datapoint_int)
            datapoint = ''
    return dataSet
def plotData():
    global signal
    global roll
    global pitch
    global yaw
    signal = Data.readline()
    signal = dataProcess(signal)
    if (len(signal)==6):
        x = signal[0] * 9.8 / 8192
        y = signal[1] * 9.8 / 8192
        z = signal[2] * 9.8 / 8192
        gyrox = signal[3] /131
        gyroy = signal[4] /131
        gyroz = signal[5] /131
        # pitch = math.atan(x/z)
        # roll = math.atan(y/z)
        # yaw = math.atan(x/y)
        # pitch += (gyrox / 131) * 0.01 * 2 * math.pi / 360
        # roll += (gyroy / 131) * 0.01 * 2 * math.pi / 360
        # yaw += (gyroz / 131) * 0.01 * 2 * math.pi / 360
        roll, pitch, yaw = complementary_filter(roll, pitch, yaw, gyrox, gyroy, gyroz, x, y, z)
        gz = math.sqrt(9.8 * 9.8 / (1 + math.pow(math.tan(roll), 2) + math.pow(math.tan(pitch), 2)))
        if z < 0:
            gz = -gz
        gx = gz * math.tan(pitch)
        gy = gz * math.tan(roll)

        signal = [x - gx, y - gy, z - gz]
        for i in range (len(data)):
            if len(data[i]) < xLength:
                data[i].append(signal[i])
            else:
                data[i][:-1] = data[i][1:]
                data[i][-1] = signal[i]
        curve1.setData(data[0], pen=pg.mkPen('g', width=3))
        curve2.setData(data[1], pen=pg.mkPen('r', width=3))
        curve3.setData(data[2], pen=pg.mkPen('b', width=3))
pitch = 0
roll = 0
yaw = 0
n = 1
sum1 = 0
sum2 = 0
sum3 = 0
# while True:
#     signal = Data.readline()
#     signal = dataProcess(signal)
#     if len(signal) == 6:
#         x = signal[0] * 9.8 / 8192
#         y = signal[1] * 9.8 / 8192
#         z = signal[2] * 9.8 / 8192
#         gyrox = signal[3] /131
#         gyroy = signal[4] /131
#         gyroz = signal[5] /131
#         # avg1 = sum1 /n
#         # avg2 = sum2 /n
#         # avg3 = sum3 /n
#         # gyrox = signal[3] / 131 - 6.688
#         # gyroy = signal[4] / 131 - 1.825
#         # gyroz = signal[5] / 131 - 0.044
#         # pitch = math.atan(x/z)
#         # roll = math.atan(y/z)
#         # yaw = math.atan(x/y)
#         # pitch += gyroy * 0.01 * math.pi/ 180
#         # roll += gyrox * 0.01 * math.pi/ 180
#         # yaw += gyroz * 0.01 * math.pi/ 180
#         roll, pitch, yaw = complementary_filter(roll, pitch, yaw, gyrox, gyroy, gyroz, x, y, z)
#         gz = math.sqrt(9.8 * 9.8 / (1 + math.pow(math.tan(roll), 2) + math.pow(math.tan(pitch), 2)))
#         if z < 0:
#             gz = -gz
#         gx = gz * math.tan(pitch)
#         gy = gz * math.tan(roll)
#         # list = [roll,pitch,yaw]
#         list = [gx, gy, gz]
#         print(list)
#     n += 1
fig1 = win.addPlot()
fig1.showGrid(x=True, y=True)
fig1.setRange(xRange=[0, xLength], padding=0)
fig1.setLabel(axis='left', text= 'g')
fig1.setLabel(axis='bottom',text='x/point')
fig1.setTitle('acceleration x')
curve1 = fig1.plot()
fig2 = win.addPlot()
fig2.showGrid(x=True, y=True)
fig2.setRange(xRange=[0, xLength], padding=0)
fig2.setLabel(axis='left', text= 'g')
fig2.setLabel(axis='bottom',text='x/point')
fig2.setTitle('acceleration y')
curve2 = fig2.plot()
fig3 = win.addPlot()
fig3.showGrid(x=True, y=True)
fig3.setRange(xRange=[0, xLength], padding=0)
fig3.setLabel(axis='left', text= 'g')
fig3.setLabel(axis='bottom',text='x/point')
fig3. setTitle('acceleration z')
curve3 = fig3.plot()
data = [np.zeros(xLength).__array__('d'), np.zeros(xLength).__array__('d'), np.zeros(xLength).__array__('d')]

timer = pg.QtCore.QTimer()
timer.timeout.connect(plotData)
win.show()
timer.start(1)
app.exec()








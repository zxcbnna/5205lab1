# This is a sample Python script.
import io

import serial
import pyqtgraph as pg
import array
import numpy as np
import math

from scipy.signal import butter, lfilter, filtfilt
from filterpy.kalman import KalmanFilter

ymin = -2
ymax = 2

# Define my low-pass filter function
def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(N=6, Wn=normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y


# Create low-pass filter parameters
cutoff_frequency = 10.0  # Adjust this value to adjust the accuracy
filter_order = 6  # Adjust this value to adjust the accuracy
cutoff_freq = 1000

# Initialize Kalman filter parameters
kf = KalmanFilter(dim_x=3, dim_z=1)  # 3 states (acceleration, velocity, position), 1 measurement (acceleration)
kf.x = np.array([0, 0, 0])  # initial state (acceleration, velocity, position)
kf.P *= 1e4  # initial uncertainty
kf.R = 1  # measurement noise
kf.Q = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) * 0.01  # process noise

# Function to filter acceleration readings using Kalman filter
def filter_acceleration(data):
    filtered_acceleration = []
    for measurement in data:
        # Predict
        kf.predict()

        # Reshape the measurement to make it 2D (1x1 array)
        measurement = np.array([[measurement]])

        # Update with measurement
        kf.update(measurement)

        # Filtered acceleration is the first element of the state vector
        filtered_acceleration.append(kf.x[0])
    return filtered_acceleration


# Complementary filter constants
alpha = 0.9  # Weight for gyroscope data
dt = 1/115200  # Sample interval (adjust as needed)

# Initialize initial angles
global roll, pitch, yaw
roll = 0.0
pitch = 0.0
yaw = 0.0
vv = [0,0,0]

# Initialize baud rate
global fs, interval
fs = 115200
interval = 1 / fs


def complementary_filter(roll, pitch, yaw, gyro_x, gyro_y, gyro_z, x, y, z):
    # Compute roll and pitch angles using complementary filter
    roll += gyro_x * dt * math.pi / 180
    pitch += gyro_y * dt * math.pi / 180
    yaw += gyro_z * dt * math.pi / 180
    # Accelerometer-based roll and pitch
    if z != 0:
        accel_pitch = math.atan(x / z)
    else:
        accel_pitch = math.copysign(1, x) * math.pi / 2
    if z != 0:
        accel_roll = math.atan(y / z)
    else:
        accel_roll = math.copysign(1, y) * math.pi / 2
    if y != 0:
        accel_yaw = math.atan(x / y)
    else:
        accel_yaw = math.copysign(1, x) * math.pi / 2
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
win = pg.GraphicsLayoutWidget()
win.setWindowTitle('demo')
win.resize(1600, 900)
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
    global v
    global vv
    signal = Data.readline()
    signal = dataProcess(signal)
    if (len(signal) == 6):
        x = signal[0] * 9.8 / 8192
        y = signal[1] * 9.8 / 8192
        z = signal[2] * 9.8 / 8192
        gyrox = signal[3] / 131
        gyroy = signal[4] / 131
        gyroz = signal[5] / 131
        # pitch = math.atan(x/z)
        # roll = math.atan(y/z)
        # yaw = math.atan(x/y)
        # pitch += (gyrox / 131) * 0.01 * 2 * math.pi / 360
        # roll += (gyroy / 131) * 0.01 * 2 * math.pi / 360
        # yaw += (gyroz / 131) * 0.01 * 2 * math.pi / 360
        vv[0] += x * dt
        vv[1] += y * dt
        vv[2] += z * dt
        for i in range(len(v)):
            if len(v[i]) < xLength:
                v[i].append(vv[i])
            else:
                v[i][:-1] = v[i][1:]
                v[i][-1] = vv[i]
        roll, pitch, yaw = complementary_filter(roll, pitch, yaw, gyrox, gyroy, gyroz, x, y, z)
        gz = math.sqrt(9.8 * 9.8 / (1 + math.pow(math.tan(roll), 2) + math.pow(math.tan(pitch), 2)))
        if z < 0:
            gz = -gz
        gx = gz * math.tan(pitch)
        gy = gz * math.tan(roll)
        
        # gx = math.sin(pitch)
        # gy = -math.sin(roll) * math.cos(pitch)
        # gz = -math.cos(roll) * math.cos(pitch)

        signal = [x - gx, y - gy, z - gz]
        for i in range(len(data)):
            if len(data[i]) < xLength:
                data[i].append(signal[i])
            else:
                data[i][:-1] = data[i][1:]
                data[i][-1] = signal[i]
        # curve1.setData(data[0], pen=pg.mkPen('g', width=3))
        # curve2.setData(data[1], pen=pg.mkPen('r', width=3))
        # curve3.setData(data[2], pen=pg.mkPen('b', width=3))

        # Apply low-pass filter to the signals
        filtered_x = butter_lowpass_filter(data[0], cutoff_freq, fs, order=filter_order)
        filtered_y = butter_lowpass_filter(data[1], cutoff_freq, fs, order=filter_order)
        filtered_z = butter_lowpass_filter(data[2], cutoff_freq, fs, order=filter_order)

        # # Apply Kalman filter
        # filtered_x = filter_acceleration(data[0])
        # filtered_y = filter_acceleration(data[1])
        # filtered_z = filter_acceleration(data[2])

        # # Calculate velocity using the trapezoidal rule for numerical integration
        # vel_x = np.cumsum(data[0]) * interval  # Numerical integration for x-axis acceleration
        # vel_y = np.cumsum(data[1]) * interval  # Numerical integration for y-axis acceleration
        # vel_z = np.cumsum(data[2]) * interval  # Numerical integration for z-axis acceleration

        # Calculate velocity using the trapezoidal rule for numerical integration
        vel_x = np.cumsum(filtered_x) * interval  # Numerical integration for x-axis acceleration
        vel_y = np.cumsum(filtered_y) * interval  # Numerical integration for y-axis acceleration
        vel_z = np.cumsum(filtered_z) * interval  # Numerical integration for z-axis acceleration

        # # Update the plots with the filtered data
        # curve1.setData(data[0], pen=pg.mkPen('g', width=3))
        # curve2.setData(data[1], pen=pg.mkPen('r', width=3))
        # curve3.setData(data[2], pen=pg.mkPen('b', width=3))

        # Update the plots with the filtered data
        curve1.setData(filtered_x, pen=pg.mkPen('g', width=3))
        curve2.setData(filtered_y, pen=pg.mkPen('r', width=3))
        curve3.setData(filtered_z, pen=pg.mkPen('b', width=3))

        # Update the plots with the filtered data
        # curve4.setData(vel_x, pen=pg.mkPen('g', width=3))
        # curve5.setData(vel_y, pen=pg.mkPen('r', width=3))
        # curve6.setData(vel_z, pen=pg.mkPen('b', width=3))

        curve4.setData(v[0], pen=pg.mkPen('g', width=3))
        curve5.setData(v[1], pen=pg.mkPen('r', width=3))
        curve6.setData(v[2], pen=pg.mkPen('b', width=3))

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
fig1.setRange(xRange=[0, xLength], yRange=[ymin, ymax], padding=0)
fig1.setLabel(axis='left', text='g')
fig1.setLabel(axis='bottom', text='x/point')
fig1.setTitle('acceleration x')
curve1 = fig1.plot()
fig2 = win.addPlot()
fig2.showGrid(x=True, y=True)
fig2.setRange(xRange=[0, xLength], yRange=[ymin, ymax], padding=0)
fig2.setLabel(axis='left', text='g')
fig2.setLabel(axis='bottom', text='x/point')
fig2.setTitle('acceleration y')
curve2 = fig2.plot()
fig3 = win.addPlot()
fig3.showGrid(x=True, y=True)
fig3.setRange(xRange=[0, xLength], yRange=[ymin, ymax], padding=0)
fig3.setLabel(axis='left', text='g')
fig3.setLabel(axis='bottom', text='x/point')
fig3.setTitle('acceleration z')
curve3 = fig3.plot()
fig4 = win.addPlot()
fig4.showGrid(x=True, y=True)
fig4.setRange(xRange=[0, xLength], padding=0)
fig4.setLabel(axis='left', text='g')
fig4.setLabel(axis='bottom', text='x/point')
fig4.setTitle('velocity x')
curve4 = fig4.plot()
fig5 = win.addPlot()
fig5.showGrid(x=True, y=True)
fig5.setRange(xRange=[0, xLength], padding=0)
fig5.setLabel(axis='left', text='g')
fig5.setLabel(axis='bottom', text='x/point')
fig5.setTitle('velocity y')
curve5 = fig5.plot()
fig6 = win.addPlot()
fig6.showGrid(x=True, y=True)
fig6.setRange(xRange=[0, xLength], padding=0)
fig6.setLabel(axis='left', text='g')
fig6.setLabel(axis='bottom', text='x/point')
fig6.setTitle('velocity z')
curve6 = fig6.plot()
data = [np.zeros(xLength).__array__('d'), np.zeros(xLength).__array__('d'), np.zeros(xLength).__array__('d')]
v = [np.zeros(xLength).__array__('d'), np.zeros(xLength).__array__('d'), np.zeros(xLength).__array__('d')]

timer = pg.QtCore.QTimer()
timer.timeout.connect(plotData)
win.show()
timer.start(1)
app.exec()







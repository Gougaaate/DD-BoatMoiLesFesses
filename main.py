import numpy as np
from drivers.imu9_driver_v2 import Imu9IO
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.encoders_driver_v2 import EncoderIO
from drivers.gps_driver_v2 import GpsIO
from line_following import followLine

imu = Imu9IO()
arduino = ArduinoIO()
encoder = EncoderIO()
gps = GpsIO()

file = open("log.txt", "w")

A = np.array([[-72943279.10031439, -28668057.74777813, 3158664.09355233],
              [-5700163.24498797, 80185389.18243794, 884174.92923037],
              [-4284025.47859625, 1854081.35680193, -67672505.82742904]])
b = np.array([-1414.5, 1552.5, -4570.5]).T

gps_points = [
    np.array([[], []]),
    np.array([[], []]),
    np.array([[], []]),
    np.array([[], []]),
    np.array([[], []])
]

point_cnt = 0

# goal_heading = input("Heading ? ")
# duration = float(input("Duration ? "))

# if goal_heading == "N":
#     goal_heading = 0
# elif goal_heading == "W":
#     goal_heading = -90
# elif goal_heading == "S":
#     goal_heading = 180
# elif goal_heading == "E":
#     goal_heading = 90

# followHeading(goal_heading, duration, imu, arduino, encoder, A, b)

while point_cnt < len(gps_points) - 1:
    followLine(file, gps, imu, arduino, encoder, A, b, gps_points, point_cnt)

file.close()

import numpy as np
from drivers.imu9_driver_v2 import Imu9IO
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.encoders_driver_v2 import EncoderIO
from heading_command import followHeading

imu = Imu9IO()
arduino = ArduinoIO()
encoder = EncoderIO()

A = np.array([[-72943279.10031439, -28668057.74777813, 3158664.09355233],
              [-5700163.24498797, 80185389.18243794, 884174.92923037],
              [-4284025.47859625, 1854081.35680193, -67672505.82742904]])
b = np.array([-1414.5, 1552.5, -4570.5]).T

goal_heading = "W"
duration = 30

if goal_heading == "N":
    goal_heading = 0
elif goal_heading == "W":
    goal_heading = -90
elif goal_heading == "S":
    goal_heading = 180
elif goal_heading == "E":
    goal_heading = 90

followHeading(arduino, imu, encoder, A, b, goal_heading, 0.5, duration)

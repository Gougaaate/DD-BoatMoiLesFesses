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

gps.set_filter_speed("0.4")
gps.get_filter_speed()
gps.set_filter_speed("0")
gps.get_filter_speed()

data_file = open("log.txt", "w")
position_file = open("position.txt", "w")

A = np.array([[-76630295.29007231, 1847525.17444344, -5194290.50608376],
              [-9907807.50228252, 41348308.99651907, 9341220.64160515],
              [-9495305.06566219, -125290.02878874, -71290538.15258537]])
b = np.array([-1478.5, 1881., -4380.]).T

gps_points = [
    np.array([[48.198943], [-3.014750]]),
    np.array([[48.199202], [-3.015000]]),
    np.array([[48.198943], [-3.014750]]),
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
    point_cnt = followLine(data_file, position_file, gps, imu, arduino,
                           encoder, A, b, gps_points, point_cnt)

data_file.close()
position_file.close()

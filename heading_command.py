import numpy as np
from get_current_heading import getHeadingSimple
from get_motors_RPM import getRPM
from drivers.gps_driver_v2 import GpsIO
from get_current_heading import degToRad
import time


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def distance(a, b):
    return np.sqrt((b[1, 0] - a[1, 0])**2 + (b[0, 0] - a[0, 0])**2)


def gpsConversion(lat, lon):
    R = 6371000  # Earth radius
    ref_lat = 48.199000  # reference latitude (random in the area)
    lat0, lon0 = 48.198943, -3.014750
    x = R * degToRad(lat - lat0) * np.cos(degToRad(ref_lat))
    y = R * degToRad(lon - lon0)
    return x, y


def getBoatPos(gps):
    gll_ok, gll_data = gps.read_gll_non_blocking()  # read gps data
    while not gll_ok:
        gll_ok, gll_data = gps.read_gll_non_blocking()  # read gps data
    print("GPS data", gll_data)

    # Convert latitude to decimal degrees format
    lat_degrees = int(gll_data[0] / 100)
    lat_minutes = gll_data[0] - lat_degrees * 100
    lat_decimal_degrees = lat_degrees + lat_minutes / 60

    # Convert longitude to decimal degrees format
    lon_degrees = int(gll_data[2] / 100)
    lon_minutes = gll_data[2] - lon_degrees * 100
    lon_decimal_degrees = -(lon_degrees + lon_minutes / 60)

    print("Boat (lat, lon) position: ", lat_decimal_degrees,
          lon_decimal_degrees)

    boat_x, boat_y = gpsConversion(lat_decimal_degrees,
                                   lon_decimal_degrees)  # convert gps to xy
    boat_pos = np.array([[boat_x], [boat_y]])  # boat position
    print("Boat (x,y) position: ", boat_pos)
    return boat_pos


def chooseRPM(wanted_rpm, goal_rpm_diff):
    goal_rpmL, goal_rpmR = wanted_rpm, wanted_rpm + abs(goal_rpm_diff)
    if goal_rpm_diff > 0:
        print("turn right")
        return goal_rpmR, goal_rpmL
    elif goal_rpm_diff <= 0:
        print("turn left")
        return goal_rpmL, goal_rpmR


def followHeading(data_file, position_file, imu, arduino, encoder, gps, A, b,
                  line_a, line_b):
    """
    follows the heading for a given duration
    :param imu: inertial measurement unit
    :param arduino: arduino
    :param encoder: encoder
    :param A: matrix A
    :param b: vector b
    :return: none
    """
    dt = 0.1  # time step
    K11, K12, K21, K22, K3 = 0.006, 0.04, 0.05, 0.08, 200  # gains
    z1, z2 = 70, 70  # integral terms
    # print("Line a: ", line_a)
    # print("Line b: ", line_b)

    boat_pos = getBoatPos(gps)  # initialize boat position
    time.sleep(0.01)
    line_angle = np.arctan2(line_b[1, 0] - line_a[1, 0],
                            line_b[0, 0] - line_a[0, 0])  # line angle

    # while np.linalg.norm(boat_pos -
    #                      line_b) > 10:  # while the boat is not at the end
    while distance(boat_pos, line_b) > 5:  # while the boat is not at the end
        print("Line points: ", line_a, line_b)
        print("Distance to point B: ", distance(boat_pos, line_b))
        boat_pos = getBoatPos(gps)  # get boat position
        time.sleep(0.01)
        line_error = np.linalg.det([[
            line_b[0, 0] - line_a[0, 0], boat_pos[0, 0] - line_a[0, 0]
        ], [line_b[1, 0] - line_a[1, 0], boat_pos[1, 0] - line_a[1, 0]
            ]]) / np.linalg.norm(line_b - line_a)  # line error

        goal_heading = line_angle - np.arctan(line_error)  # desired heading
        print("Goal heading", goal_heading)

        heading = getHeadingSimple(imu, A, b)  # current heading
        print("Current heading", heading)
        heading_error = goal_heading - heading
        if heading_error > 70:
            print("turn right")
            command_pwmL, command_pwmR = 60, 0
            rpmL, rpmR = getRPM(encoder)  # read the RPM of the motors
            goal_rpmL, goal_rpmR = 0, 0
        elif heading_error < -70:
            print("turn left")
            command_pwmL, command_pwmR = 0, 60
            rpmL, rpmR = getRPM(encoder)  # read the RPM of the motors
            goal_rpmL, goal_rpmR = 0, 0
        else:
            goal_rpm_diff = K3 * heading_error  # desired rpm difference
            wanted_rpm = 3000  # pivot rpm value (constant)
            goal_rpmL, goal_rpmR = chooseRPM(
                wanted_rpm, goal_rpm_diff
            )  # choose the rpm of the motors to turn left or right
            rpmL, rpmR = getRPM(encoder)  # read the RPM of the motors
            e1, e2 = goal_rpmL - rpmL, goal_rpmR - rpmR  # error terms
            z1 += e1 * dt
            z2 += e2 * dt

            command_pwmL = K11 * e1 + K21 * z1
            command_pwmR = K12 * e2 + K22 * z2

        if command_pwmL > 255:
            command_pwmL = 255
        elif command_pwmL < 0:
            command_pwmL = 0
        if command_pwmR > 255:
            command_pwmR = 255
        elif command_pwmR < 0:
            command_pwmR = 0

        data_to_write = [
            rpmL, rpmR, goal_rpmL, goal_rpmR, command_pwmL, command_pwmR,
            heading,
            abs(heading_error)
        ]

        for data in data_to_write:
            data_file.write(str(data) + " ")
        data_file.write("\n")

        position_file.write(
            str(boat_pos[0, 0]) + " " + str(boat_pos[1, 0]) + "\n")

        gps = GpsIO()
        arduino.send_arduino_cmd_motor(command_pwmL, command_pwmR)

    arduino.send_arduino_cmd_motor(0, 0)  # turn off motors

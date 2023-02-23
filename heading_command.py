import numpy as np
# import pyproj
from get_current_heading import getHeadingSimple
from get_motors_RPM import getRPM
from get_current_heading import degToRad


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def manual_conversion(lat, long):
    rho = 6371000
    lat0 = np.pi / 180 * 48.198943
    long0 = np.pi / 180 * -3.014750
    lat = np.pi / 180 * lat
    long = np.pi / 180 * long
    xt = rho * np.cos(lat) * (lat - lat0)
    yt = rho * (long - long0)
    return xt, yt


def gps_conversion(lat, lon):
    R = 6371000  # Earth radius
    ref_lat = 48.199000  # reference latitude (random in the area)
    lat0, lon0 = 48.198943, -3.014750
    x = R * (lon - lon0) * np.cos(degToRad(ref_lat))
    y = R * (lat - lat0)
    return x, y


# def gps_to_xy(lat, lon):
#     # Define the projection system you want to use
#     project = pyproj.Proj(proj='utm', zone='30T', ellps='WGS84')

#     # Define the GPS coordinates of the origin point
#     lat0, lon0 = 48.198943, -3.014750

#     # Convert the origin GPS coordinates to UTM coordinates
#     x0, y0 = project(lon0, lat0)

#     # Convert the GPS coordinates to x, y coordinates
#     x, y = project(lon, lat)

#     # Calculate the relative UTM coordinates with respect to the origin
#     rel_x = x - x0
#     rel_y = y - y0

#     return rel_x, rel_y


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
    :param goal_heading: desired heading
    :param duration: duration of the race
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

    gps_ok, gps_data = gps.read_gll_non_blocking()  # read gps data
    boat_lat, boat_lon = gps_data[0], gps_data[2]  # get boat position
    boat_x, boat_y = gps_conversion(boat_lat, boat_lon)  # convert gps to xy
    boat_pos = np.array([[boat_x], [boat_y]])  # boat position
    line_error = np.linalg.det([[
        line_b[0, 0] - line_a[0, 0], boat_pos[0, 0] - line_a[0, 0]
    ], [line_b[1, 0] - line_a[1, 0], boat_pos[1, 0] - line_a[1, 0]
        ]]) / np.linalg.norm(line_b - line_a)  # line error
    line_angle = np.arctan2(line_b[1, 0] - line_a[1, 0],
                            line_b[0, 0] - line_a[0, 0])  # line angle
    goal_heading = line_angle - np.arctan(line_error)  # desired heading

    # global_init_time = time.time()
    # while time.time() - global_init_time < duration:
    while np.linalg.norm(boat_pos -
                         line_b) < 1:  # while the boat is not at the end
        # init_time = time.time()
        heading = getHeadingSimple(imu, A, b)  # current heading
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

        position_file.write(str(boat_x) + " " + str(boat_y) + "\n")

        arduino.send_arduino_cmd_motor(command_pwmL, command_pwmR)

    arduino.send_arduino_cmd_motor(0, 0)  # turn off motors

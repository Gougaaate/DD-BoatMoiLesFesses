import time
import numpy as np
from get_current_heading import getHeadingSimple
from get_motors_RPM import getRPM


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def chooseRPM(wanted_rpm, goal_rpm_diff):
    goal_rpmL, goal_rpmR = wanted_rpm, wanted_rpm + abs(goal_rpm_diff)
    if goal_rpm_diff < 0:
        # print("go right")
        return goal_rpmR, goal_rpmL
    elif goal_rpm_diff >= 0:
        # print("go left")
        return goal_rpmL, goal_rpmR


def followHeading(goal_heading, duration, imu, arduino, encoder, A, b):
    """
    follows the heading for a given duration
    :param encoder:
    :param arduino:
    :param imu:
    :param goal_heading:
    :param duration: duration of the race
    :return: none
    """
    cnt = 0
    file = open("log.txt", "w")

    dt = 0.01  # time step
    K11, K12, K21, K22, K3 = 0., 0., 0.06, 0.06, 650  # gains
    z1, z2 = 70, 70  # integral terms
    global_init_time = time.time()
    while time.time() - global_init_time < duration:
        init_time = time.time()
        heading = getHeadingSimple(imu, A, b)  # current heading
        goal_rpm_diff = K3 * sawtooth(
            np.pi * goal_heading / 180 -
            np.pi * heading / 180)  # sawtooth takes radians as input
        wanted_rpm = 3000  # pivot rpm value (constant)
        goal_rpmL, goal_rpmR = chooseRPM(
            wanted_rpm, goal_rpm_diff
        )  # choose the rpm of the motors to turn left or right
        rpmL, rpmR = getRPM(encoder)  # read the RPM of the motors
        e1, e2 = goal_rpmL - rpmL, goal_rpmR - rpmR  # error terms

        z1 += e1 * dt
        z2 += e2 * dt
        command_rpmL = K11 * e1 + K21 * z1
        command_rpmR = K12 * e2 + K22 * z2

        # print("###################################")
        # print("rpmL : ", w1, "rpmR : ", w2)
        # print("omega bar : ", wbar1, wbar2)
        # print("z : ", z1, z2)
        # print("u : ", u1, u2)
        # print("psi : ", psi)

        data_to_write = [
            rpmL, rpmR, command_rpmL, command_rpmR, heading,
            abs(goal_heading - heading)
        ]

        for data in data_to_write:
            file.write(str(data) + " ")
        file.write("\n")
        arduino.send_arduino_cmd_motor(50, 50)

        # arduino.send_arduino_cmd_motor(command_rpmL, command_rpmR)

        end_time = time.time()
        if end_time - init_time < dt:
            time.sleep(dt - (end_time - init_time))
        cnt += 1
        print(cnt)
    arduino.send_arduino_cmd_motor(0, 0)  # turn off motors
    file.close()

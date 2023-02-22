import time
import numpy as np
from get_current_heading import getHeadingSimple, degToRad
from get_motors_RPM import getRPM


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def chooseRPM(wanted_rpm, goal_rpm_diff):
    goal_rpmL, goal_rpmR = wanted_rpm, wanted_rpm + abs(goal_rpm_diff)
    if goal_rpm_diff < 0:
        print("turn right")
        return goal_rpmR, goal_rpmL
    elif goal_rpm_diff >= 0:
        print("turn left")
        return goal_rpmL, goal_rpmR


def adjustPWM(command_pwmL, command_pwmR, command_rpmL, command_rpmR, encoder):
    if command_pwmL > 255:
        command_pwmL = 255
    elif command_pwmL < 0:
        command_pwmL = 0
    elif command_rpmL - getRPM(encoder)[0] > 50:
        while command_rpmL - getRPM(encoder)[0] > 50:
            command_pwmL += 5
    elif command_rpmL - getRPM(encoder)[0] < -50:
        while command_rpmL - getRPM(encoder)[0] < -50:
            command_pwmL -= 5

    if command_pwmR > 255:
        command_pwmR = 255
    elif command_pwmR < 0:
        command_pwmR = 0
    elif command_rpmR - getRPM(encoder)[1] > 50:
        while command_rpmR - getRPM(encoder)[1] > 50:
            command_pwmR += 5
    elif command_rpmR - getRPM(encoder)[1] < -50:
        while command_rpmR - getRPM(encoder)[1] < -50:
            command_pwmR -= 5

    return command_pwmL, command_pwmR


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
    loop_cnt = 0
    file = open("log.txt", "w")

    dt = 0.1  # time step
    K11, K12, K21, K22, K3 = 0.1, 0.1, 0.05, 0.05, 650  # gains
    z1, z2 = 70, 70  # integral terms
    command_pwmL, command_pwmR = 80, 80  # pwm values
    global_init_time = time.time()
    while time.time() - global_init_time < duration:
        init_time = time.time()
        heading = getHeadingSimple(imu, A, b)  # current heading
        goal_rpm_diff = K3 * sawtooth(
            degToRad(goal_heading) -
            degToRad(heading))  # sawtooth takes radians as input
        wanted_rpm = 3000  # pivot rpm value (constant)
        goal_rpmL, goal_rpmR = chooseRPM(
            wanted_rpm, goal_rpm_diff
        )  # choose the rpm of the motors to turn left or right
        rpmL, rpmR = getRPM(encoder)  # read the RPM of the motors
        e1, e2 = goal_rpmL - rpmL, goal_rpmR - rpmR  # error terms
        z1 += e1 * dt
        z2 += e2 * dt
        command_rpmL = K11 * e1  # + K21 * z1
        command_rpmR = K12 * e2  # + K22 * z2

        command_pwmL, command_pwmR = adjustPWM(command_pwmL, command_pwmR,
                                               command_rpmL, command_rpmR,
                                               encoder)

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

        arduino.send_arduino_cmd_motor(command_pwmL, command_pwmR)

        end_time = time.time()
        if end_time - init_time < dt:
            time.sleep(dt - (end_time - init_time))

        loop_cnt += 1
        print(loop_cnt)

    arduino.send_arduino_cmd_motor(0, 0)  # turn off motors
    file.close()

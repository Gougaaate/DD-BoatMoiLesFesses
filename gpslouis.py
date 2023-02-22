from drivers.gps_driver_v2 import GpsIO
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.imu9_driver_v2 import Imu9IO
from drivers.encoders_driver_v2 import EncoderIO
import numpy as np
from get_current_heading import getHeadingSimple
from get_motors_RPM import getRPM

gps = GpsIO()

Ainv = np.array([[-72943279.10031439, -28668057.74777813, 3158664.09355233],
                 [-5700163.24498797, 80185389.18243794, 884174.92923037],
                 [-4284025.47859625, 1854081.35680193, -67672505.82742904]])
binv = np.array([-1414.5, 1552.5, -4570.5]).T


def dist_entre_deux_points(a, b):
    return np.sqrt((b[1]-a[1])**2+(b[0]-a[0])**2)

def conversion_manuelle(lat, long):
    rho = 6371000
    latm = np.pi / 180 * 48.198943
    longm = np.pi / 180 * -3.014750
    lat = np.pi / 180 * lat
    long = np.pi / 180 * long
    xt = rho * np.cos(lat) * (lat - latm)
    yt = rho * (long - longm)
    return xt, yt


def chooseRPM(wanted_rpm, goal_rpm_diff):
    goal_rpmL, goal_rpmR = wanted_rpm, wanted_rpm + abs(goal_rpm_diff)
    if goal_rpm_diff > 0:
        print("turn right")
        return goal_rpmR, goal_rpmL
    elif goal_rpm_diff <= 0:
        print("turn left")
        return goal_rpmL, goal_rpmR


def estalabouee(nom, xy):
    if nom == "A":
        xf, yf = conversion_manuelle(48.199276, -3.014887)
    if nom == "B":
        xf, yf = conversion_manuelle(48.199508, -3.015295)
    if nom == "C":
        xf, yf = conversion_manuelle(48.199184, -3.015283)
    xfyf = [xf, yf]
    if dist_entre_deux_points(xy, xfyf) > 10:
        return False
    else:
        return True


def sawtooth(x):
    return (x + np.pi) % (
            2 * np.pi) - np.pi  # or equivalently   2*np.arctan(np.tan(x/2))


def suivi_gps(arduino, imu, matA, vecb, vitesse):
    Ax, Ay = conversion_manuelle(48.199508, -3.015295)
    Bx, By = conversion_manuelle(48.199184, -3.015283)
    a = np.array([[Ax], [Ay]])
    b = np.array([[Bx], [By]])
    gpsok, gpsdata = gps.read_gll_non_blocking()
    xi, yi = conversion_manuelle(gpsdata[0], gpsdata[2])
    phi = np.arctan2(a[1, 0] - yi, a[0, 0] - xi)
    xiyi = [xi, yi]
    while estalabouee("A", xiyi)==False:
        xiyi = np.array([[xi], [yi]])
        gpsok, gpsdata = gps.read_gll_non_blocking()
        x, y = conversion_manuelle(gpsdata[0], gpsdata[2])
        m = np.array([[x], [y]])
        u = m - xiyi
        att = m - a
        v = (a - xiyi) / np.linalg.norm(a - xiyi)
        A = np.hstack((u, v))
        e = np.linalg.det(A)
        capdes = phi - np.arctan(e)
        file = open("log.txt", "w")
        dt = 0.1  # time step
        K11, K12, K21, K22, K3 = 0.006, 0.04, 0.05, 0.08, 200  # gains
        z1, z2 = 70, 70  # integral terms
        heading=getHeadingSimple(imu,matA,vecb)
        while abs(capdes - heading) > 20:
            heading = getHeadingSimple(imu, matA, vecb)  # current heading
            heading_error = capdes - heading
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
                heading, abs(heading_error)
            ]

            for data in data_to_write:
                file.write(str(data) + " ")
            file.write("\n")
            arduino.send_arduino_cmd_motor(command_pwmL, command_pwmR)
    arduino.send_arduino_cmd_motor(0, 0)  # turn off motors
    file.close()


if __name__ == "__main__":
    arduino = ArduinoIO()
    encoder = EncoderIO()
    imu = Imu9IO()
    suivi_gps(arduino,imu,Ainv,binv,80)

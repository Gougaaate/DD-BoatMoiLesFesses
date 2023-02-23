from drivers.gps_driver_v2 import GpsIO
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.imu9_driver_v2 import Imu9IO
from drivers.encoders_driver_v2 import EncoderIO
import numpy as np
from heading_command import followHeading

gps = GpsIO()

Ainv = np.array([[-72943279.10031439, -28668057.74777813, 3158664.09355233],
                 [-5700163.24498797, 80185389.18243794, 884174.92923037],
                 [-4284025.47859625, 1854081.35680193, -67672505.82742904]])
binv = np.array([-1414.5, 1552.5, -4570.5]).T


def dist_entre_deux_points(a, b):
    return np.sqrt((b[1] - a[1]) ** 2 + (b[0] - a[0]) ** 2)


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
    else:
        xf, yf = conversion_manuelle(48.199184, -3.015283)
    xfyf = [xf, yf]
    if dist_entre_deux_points(xy, xfyf) > 10:
        return False
    else:
        return True


def sawtooth(x):
    return (x + np.pi) % (
            2 * np.pi) - np.pi  # or equivalently   2*np.arctan(np.tan(x/2))


def suivi_gps(arduino, imu):
    Ax, Ay = conversion_manuelle(48.199508, -3.015295)
    Bx, By = conversion_manuelle(48.199184, -3.015283)
    a = np.array([[Ax], [Ay]])
    b = np.array([[Bx], [By]])
    gpsok, gpsdata = gps.read_gll_non_blocking()
    xi, yi = conversion_manuelle(gpsdata[0], gpsdata[2])
    phi = np.arctan2(a[1, 0] - yi, a[0, 0] - xi)
    xiyi = [xi, yi]
    while not estalabouee("A", xiyi):
        xiyi = np.array([[xi], [yi]])
        gpsok, gpsdata = gps.read_gll_non_blocking()
        x, y = conversion_manuelle(gpsdata[0], gpsdata[2])
        m = np.array([[x], [y]])
        u = m - xiyi
        v = (a - xiyi) / np.linalg.norm(a - xiyi)
        A = np.hstack((u, v))
        e = np.linalg.det(A)
        capdes = phi - np.arctan(e)
        followHeading(capdes, 10000, imu, arduino, encoder, Ainv, binv)


if __name__ == "__main__":
    arduino = ArduinoIO()
    encoder = EncoderIO()
    imu = Imu9IO()
    suivi_gps(arduino, imu)

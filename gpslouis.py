from drivers.gps_driver_v2 import GpsIO
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.imu9_driver_v2 import Imu9IO
from drivers.encoders_driver_v2 import EncoderIO
from heading_command import followHeading
import numpy as np

gps = GpsIO()

Ainv = np.array([[-72943279.10031439, -28668057.74777813, 3158664.09355233],
                 [-5700163.24498797, 80185389.18243794, 884174.92923037],
                 [-4284025.47859625, 1854081.35680193, -67672505.82742904]])
binv = np.array([-1414.5, 1552.5, -4570.5]).T


def dist_entre_deux_points(a, b):
    phia = a[0]
    lambdaa = a[2]
    phib = b[0]
    lambdab = b[2]
    dist = 2 * np.arcsin(
        np.sqrt(
            np.sin(0.5 * (phib - phia))**2 +
            np.cos(phia) * np.cos(phib) * np.sin(0.5 *
                                                 (lambdab - lambdaa))**2))
    return dist


def conversion_manuelle(lat, long):
    rho = 6371000
    latm = np.pi / 180 * 48.198943
    longm = np.pi / 180 * -3.014750
    lat = np.pi / 180 * lat
    long = np.pi / 180 * long
    xt = rho * np.cos(lat) * (lat - latm)
    yt = rho * (long - longm)
    return xt, yt


def suivi_gps():
    arduino.send_arduino_cmd_motor(0, 0)
    Ax, Ay = conversion_manuelle(48.199508, -3.015295)
    Bx, By = conversion_manuelle(48.199184, -3.015283)
    A = np.array([[Ax], [Ay]])
    B = np.array([[Bx], [By]])
    phi = np.arctan2(By - Ay, Bx - Ax)
    gll_ok, gll_data = gps.read_gll_non_blocking()
    x, y = gll_data
    XY = np.array([[x], [y]])
    u = XY - A
    v = (B - A) / np.linalg.norm(B - A)
    A = np.hstack((u, v))
    e = np.linalg.det(A)
    capdes = phi - np.arctan(e)
    followHeading(capdes, 30, imu, arduino, encoder, Ainv, binv)


if __name__ == "__main__":
    arduino = ArduinoIO()
    encoder = EncoderIO()
    imu = Imu9IO()

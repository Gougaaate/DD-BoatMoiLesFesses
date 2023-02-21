import time
import numpy as np
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.imu9_driver_v2 import Imu9IO
from drivers.encoders_driver_v2 import EncoderIO
from getRPM import getRPM

A_inv = np.array([[-72943279.10031439, -28668057.74777813, 3158664.09355233],
                  [-5700163.24498797, 80185389.18243794, 884174.92923037],
                  [-4284025.47859625, 1854081.35680193, -67672505.82742904]])
b = np.array([-1414.5, 1552.5, -4570.5]).T


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def followHeading(self, psibar, T):
    """
    follows the heading psi
    :param psi: angle corresponding to the heading
    :return: nothing
    """
    dt = 0.5
    K11, K12, K21, K22, K3 = 0., 0., 0.06, 0.06, 650
    z1, z2 = 70, 70
    k = 0
    T0 = time.time()
    while time.time() - T0 < T:
        k += 1
        t0 = time.time()

        psi = self.get_angle()
        deltawbar = K3 * sawtooth(np.pi * psibar / 180 - np.pi * psi /
                                  180)  #sawtooth prend des radians
        #deltawbar = 0 #provisoirement
        wbarbar = 3000  #consigne pivot
        wbar1, wbar2 = self.choosew(
            wbarbar, deltawbar)  #détermine si on va à gauche ou à droite
        w1, w2 = self.omega2()  #retourne les mesures des rpm
        e1, e2 = wbar1 - w1, wbar2 - w2

        z1 += e1 * dt  #termes intégrateurs
        z2 += e2 * dt
        u1 = K11 * e1 + K21 * z1
        u2 = K12 * e2 + K22 * z2

        # print("###################################")
        # print("rpmL : ", w1, "rpmR : ", w2)
        # print("omega bar : ", wbar1, wbar2)
        # print("z : ", z1, z2)
        # print("u : ", u1, u2)
        # print("psi : ", psi)

        arduino.send_arduino_cmd_motor(u1, u2)

        t1 = time.time()
        if t1 - t0 < dt:
            time.sleep(dt - (t1 - t0))

    self.ard.send_arduino_cmd_motor(0, 0)  #on coupe les moteurs


if __name__ == "__main__":
    imu = Imu9IO()
    arduino = ArduinoIO()
    encoder = EncoderIO()

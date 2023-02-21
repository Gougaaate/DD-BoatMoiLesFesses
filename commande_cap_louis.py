from math import atan2
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.imu9_driver_v2 import Imu9IO
from drivers.encoders_driver_v2 import EncoderIO
import numpy as np
import time
from getRPM import getRPM

cuml, cumr = 0, 0
last_wl, last_wr = 0, 0
def run(arduino, imu, objectif, vitesse, temps):
    A_inv = np.array([[-72943279.10031439, -28668057.74777813, 3158664.09355233],
                      [-5700163.24498797, 80185389.18243794, 884174.92923037],
                      [-4284025.47859625, 1854081.35680193, -67672505.82742904]])
    b = np.array([-1414.5, 1552.5, -4570.5]).T

    k3 = 5

    if objectif == "N":
        objectif = 0
    elif objectif == "O":
        objectif = -90
    elif objectif == "S":
        objectif = 180
    elif objectif == "E":
        objectif = 90
    print("Arduino status is", arduino.get_arduino_status())
    print("RC channel is", arduino.get_arduino_rc_chan())

    print("Press Ctrl+C to get out!")
    try:
        tinit = time.time()
        tfin = time.time()
        while tfin - tinit <= temps:
            temps_reel = time.time()
            temps_boucle_cap = time.time()
            w1, w2, cap, e = commande_en_cap(arduino, imu, A_inv, b, k3,
                                             objectif, vitesse)

            while temps_reel - temps_boucle_cap < 2:
                temps_boucle_encoder = time.time()

                # regule(encoder, arduino, w1, w2)
                print("Cap : ", cap)
                print("Erreur cap : ", e)
                temps_reel = time.time()

                while temps_reel - temps_boucle_encoder < 0.5:
                    temps_reel = time.time()
            tfin = time.time()

        arduino.send_arduino_cmd_motor(0, 0)

    except KeyboardInterrupt:
        print("Stop")

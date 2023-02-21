from math import atan2
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.imu9_driver_v2 import Imu9IO
from drivers.encoders_driver_v2 import EncoderIO
import numpy as np
import time

document = "~/ddboat-titannick/Values/speed_motor.txt"
cuml, cumr = 0, 0
last_wl, last_wr = 0, 0


def run(arduino, encoder, imu, objectif, vitesse, temps):
    A_inv = np.linalg.inv(imu.A)
    b = imu.b
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

                regule(encoder, arduino, w1, w2)
                print("Cap : ", cap)
                print("Erreur cap : ", e)
                temps_reel = time.time()

                while temps_reel - temps_boucle_encoder < 0.5:
                    temps_reel = time.time()
            tfin = time.time()

        arduino.send_arduino_cmd_motor(0, 0)

    except KeyboardInterrupt:
        print("Stop")


def sawtooth(x):
    return (x + np.pi) % (
        2 * np.pi) - np.pi  # or equivalently   2*np.arctan(np.tan(x/2))


def regule(encoder, controller, cw_left, cw_right):
    global cuml, cumr, last_wl, last_wr

    w_left, w_right = encoder.get_speed()
    dt = 0.5

    err_left = cw_left - w_left
    err_right = cw_right - w_right

    cuml = cuml + err_left * dt
    cumr = cumr + err_right * dt

    if cuml * 0.2 >= 100:
        cuml = 0
    elif cumr * 0.2 >= 100:
        cumr = 0

    cmdl = max(0, min(255, 1.2 * err_left + 0.2 * cuml))
    cmdr = max(0, min(255, 1.2 * err_right + 0.2 * cumr))

    print("commande moteur gauche : {}\t droite: {}".format(
        np.round(cmdl, 5), np.round(cmdr, 5)))
    print("consigne moteur gauche : {}\t droite: {}".format(
        np.round(cw_left, 5), np.round(cw_right, 5)))
    print("vitesse relevée gauche : {}\t droite: {}".format(
        np.round(w_left, 5), np.round(w_right, 5)))
    print("erreur gauche          : {}\t droite: {}".format(
        np.round(err_left, 5), np.round(err_right, 5)))
    controller.send_arduino_cmd_motor(cmdl, cmdr)

    last_wl, last_wr = w_left, w_right


def commande_en_cap(arduino, imu, A_inv, b, k3, objectif, vitesse):
    y = A_inv @ (np.array(imu.read_mag_raw()) + b).T
    cap = atan2(y[1, 0], y[0, 0]) * (180 / np.pi)
    e = float(objectif) - cap
    if e > 180:
        e = -360 + e
    elif e < -180:
        e = 360 + e

    w1 = (float(vitesse) + k3 * sawtooth(e * np.pi / 180)) / 2
    w2 = (float(vitesse) - k3 * sawtooth(e * np.pi / 180)) / 2

    if w1 < 0:
        w1 = 0
    elif w1 > float(vitesse):
        w1 = float(vitesse)
    if w2 < 0:
        w2 = 0
    elif w2 > float(vitesse):
        w2 = float(vitesse)

    print("arduino motors rc", arduino.get_arduino_cmd_motor())
    print("ENSV: ", arduino.get_arduino_energy_saver())
    print("Cap: ", cap)
    print("Erreur: ", e)
    print("w1 : ", w1)
    print("w2 : ", w2)
    return w1, w2, cap, e


if __name__ == "__main__":
    arduino = ArduinoIO()
    encoder = EncoderIO()
    imu = Imu9IO()
    cap = input("Rentrez le cap")
    vitesse = input("Rentrez la vitesse")
    temps = input("Rentrez le temps en s")
    run(arduino, encoder, imu, cap, vitesse, temps)

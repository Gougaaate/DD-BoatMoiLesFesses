from drivers.encoders_driver_v2 import EncoderIO
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.imu9_driver_v2 import Imu9IO
import sys
import numpy as np

sys.path.append("./drivers")

imu = Imu9IO()
arduino = ArduinoIO()
encoder = EncoderIO()


def delta_odo(odo1, odo0):
    '''
    computes the difference between 2 encoder values
    '''
    dodo = odo1 - odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


def getRPM(self):
    # time.sleep(5.0)
    delta_t = 3.0
    n_t = int(round(delta_t * 10))
    self.enc.set_older_value_delay_v2(n_t)

    st1, st0 = self.enc.get_last_and_older_values_v2()
    data_encoders0 = np.array(st0.split(',')).astype(np.float)
    data_encoders1 = np.array(st1.split(',')).astype(np.float)

    timeAcq0 = data_encoders0[0] / 10.0
    sensLeft0 = data_encoders0[2]
    sensRight0 = data_encoders0[1]
    posLeft0 = data_encoders0[4]
    posRight0 = data_encoders0[3]

    timeAcq1 = data_encoders1[0] / 10.0
    sensLeft1 = data_encoders1[2]
    sensRight1 = data_encoders1[1]
    posLeft1 = data_encoders1[4]
    posRight1 = data_encoders1[3]

    delta_t = timeAcq1 - timeAcq0
    rpmL = abs(delta_odo(posLeft1, posLeft0) / 8.0 / delta_t * 60.0)
    rpmR = abs(delta_odo(posRight1, posRight0) / 8.0 / delta_t * 60.0)

    # print("RPM Left", rpmL, "RPM Right", rpmR)

    return abs(rpmL), abs(rpmR)


arduino.send_arduino_cmd_motor(80, 80)
while True:
    encoder.get_last_value_v2()

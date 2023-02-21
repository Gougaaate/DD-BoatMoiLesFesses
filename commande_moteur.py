from drivers.encoders_driver_v2 import EncoderIO
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.imu9_driver_v2 import Imu9IO
import sys
sys.path.append("./drivers")

imu = Imu9IO()
arduino = ArduinoIO()
encoder = EncoderIO()

arduino.send_arduino_cmd_motor(80,80)
while True:
    encoder.get_last_value_v2()



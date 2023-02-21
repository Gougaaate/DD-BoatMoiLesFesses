from drivers.encoders_driver_v2 import EncoderIO
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.imu9_driver_v2 import Imu9IO
imu = Imu9IO()
arduino = ArduinoIO()
encoder = EncoderIO()

arduino.send_arduino_cmd_motor(80,80)
while True:
    arduino.get_arduino_cmd_motor()


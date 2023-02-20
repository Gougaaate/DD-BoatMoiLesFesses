from tc74_driver_v2 import TempTC74IO
import time

temperature = TempTC74IO()

for k in range(50):
    time.sleep(1)
    print(temperature.read_temp())

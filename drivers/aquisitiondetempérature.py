import struct
import time
import sys
import math
import i2creal as i2c
import tc74_driver_v2
temperature=tc74_driver_v2.TempTC74IO()
f=open("relevé de température.txt","w")
cfg_left,cfg_right=temperature.get_config()
to=time.time()
tmax=30
f.write(time.time(())
while time.time()-to<= tmax:
    g,d=temperature.read_temp()
    f.writelines(g,d)
    time.sleep(1.0)
f.close()
    

    
    
    
    


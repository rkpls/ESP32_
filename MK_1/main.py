
# ---------- SETUP ----------
from machine import I2C         
from pyb import I2C
import epaper2in9b


def write_epaper_qr():
    pass
def write_epaper_logo():
    pass


i2c_display = I2C(0, I2C.CONTROLLER)
I2C.init(I2C.CONTROLLER,baudreate=400000)
print(I2C.scan())


write_epaper_qr()
write_epaper_logo()



import adler
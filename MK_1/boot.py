
# ---------- BOOT ----------
from machine import Pin, SoftI2C

pin_SDA2 = 6
pin_SCL2 = 7
pin_SDA3 = 15
pin_SCL3 = 16
pin_SDA4 = 17
pin_SCL4 = 18
pin_SDA5 = 8
pin_SCL5 = 9
pin_SDA1 = 1
pin_SCL1 = 2

def i2c_SPI_setup():
    try:
        global i2c1, i2c2, i2c3, i2c4, i2c5
        i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
        i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)
        i2c3 = SoftI2C(scl=Pin(pin_SCL3), sda=Pin(pin_SDA3), freq=100000)
        i2c4 = SoftI2C(scl=Pin(pin_SCL4), sda=Pin(pin_SDA4), freq=100000)
        i2c5 = SoftI2C(scl=Pin(pin_SCL5), sda=Pin(pin_SDA5), freq=100000)
    except:
        print("could not innit i2c")

def scan_i2c():
    print('Scan i2c')
    devices1 = i2c1.scan()
    devices2 = i2c2.scan()
    devices3 = i2c3.scan()
    devices4 = i2c4.scan()
    devices5 = i2c5.scan()

    for device1 in devices1:  
        print("Hex address: ",hex(device1))
    if len(devices1) == 0:
        print("No i2c device !")
    else:
        print('i2c devices found:',len(devices2))

    for device2 in devices2:  
        print("Hex address: ",hex(device2))
    if len(devices2) == 0:
        print("No i2c device !")
    else:
        print('i2c devices found:',len(devices2))

    for device3 in devices3:  
        print("Hex address: ",hex(device3))
    if len(devices3) == 0:
        print("No i2c device !")
    else:
        print('i2c devices found:',len(devices3))
        
    for device4 in devices4:  
        print("Hex address: ",hex(device4))
    if len(devices4) == 0:
        print("No i2c device !")
    else:
        print('i2c devices found:',len(devices4))

    for device5 in devices5:  
        print("Hex address: ",hex(device5))
    if len(devices5) == 0:
        print("No i2c device !")
    else:
        print('i2c devices found:',len(devices5))

i2c_SPI_setup()
scan_i2c()

import main
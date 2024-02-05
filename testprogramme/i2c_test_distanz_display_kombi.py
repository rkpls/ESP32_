
from machine import Pin, SoftI2C
from time import sleep_ms
from vl53l0x import VL53L0X
import sh1106


pin_SDA1 = 1
pin_SCL1 = 2
pin_SDA2 = 6
pin_SCL2 = 7
pin_SDA3 = 15
pin_SCL3 = 16
pin_SDA4 = 17
pin_SCL4 = 18
pin_SDA5 = 8
pin_SCL5 = 9

dist1 = 0
dist2 = 0

i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)
i2c3 = SoftI2C(scl=Pin(pin_SCL3), sda=Pin(pin_SDA3), freq=100000)
i2c4 = SoftI2C(scl=Pin(pin_SCL4), sda=Pin(pin_SDA4), freq=100000)
i2c5 = SoftI2C(scl=Pin(pin_SCL5), sda=Pin(pin_SDA5), freq=100000)


tof1 = VL53L0X(i2c3)
tof2 = VL53L0X(i2c4)
display2 = sh1106.SH1106_I2C(128, 64, i2c2, Pin(0), 0x3c)

tof1.set_measurement_timing_budget(10000)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[0], 12)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[1], 8)

display2.fill(0)
display2.flip()

while True:

    dist1 = str(tof1.read())
    dist2 = str(tof2.read())
    sleep_ms(50)
    display2.fill(0)
    display2.text("Display 2", 0, 0, 1)
    display2.text("Vorne / Oben", 0, 16, 1)
    display2.text(dist1, 0, 32, 1)
    display2.text(dist2, 0, 48, 1)
    display2.show()




from machine import Pin, SoftI2C
from time import sleep_ms
from vl53l0x import VL53L0X
import sh1106


pin_SDA1 = 15
pin_SCL1 = 16
pin_SDA2 = 17
pin_SCL2 = 18
#pin_SDA0 = 8
#pin_SCL = 9

#i2c0 = SoftI2C(scl=Pin(pin_SCL), sda=Pin(pin_SDA0), freq=100000)
i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)

tof1 = VL53L0X(i2c1)
tof2 = VL53L0X(i2c2)

display1 = sh1106.SH1106_I2C(128, 64, i2c1, Pin(0), 0x3c)
display2 = sh1106.SH1106_I2C(128, 64, i2c2, Pin(0), 0x3c)

tof1.set_measurement_timing_budget(10000)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[0], 12)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[1], 8)

tof2.set_measurement_timing_budget(10000)
tof2.set_Vcsel_pulse_period(tof2.vcsel_period_type[0], 12)
tof2.set_Vcsel_pulse_period(tof2.vcsel_period_type[1], 8)

display1.fill(0)
display2.fill(0)

while True:

    dist1 = str(tof1.read())
    dist2 = str(tof2.read())
    sleep_ms(50)
    display1.fill(0)
    display1.text("Display 1", 0, 0, 1)
    display1.text("Sensor 1", 0, 16, 1)
    display1.text(dist1, 0, 32, 1)
    display1.show()
    display2.fill(0)
    display2.text("Display 2", 0, 0, 1)
    display2.text("Sensor 2", 0, 16, 1)
    display2.text(dist2, 0, 32, 1)
    display2.show()

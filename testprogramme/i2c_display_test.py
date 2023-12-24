
from machine import Pin, SoftI2C
import machine
from time import sleep_ms
import utime
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

width=128
height=64

i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)

display2 = sh1106.SH1106_I2C(128, 64, i2c2, Pin(0), 0x3c)


display2.sleep(False)
display2.flip()

display2.fill(0) # clear to black
display2.text('Adler Display', 0, 0, 1) # at x=0, y=0, white on black
display2.hline(0, 9, 127, 1)
display2.hline(0, 30, 127, 1)
display2.vline(0, 10, 32, 1)
display2.vline(127, 10, 32, 1)

for i in range(0, 118):
    display2.fill_rect(i,10, 10, 10, 1)
    display2.fill_rect(10, 21, 30, 8, 0)
    display2.text(str(i), 10, 21, 1)
    display2.show() # update display

display1 = sh1106.SH1106_I2C(128, 64, i2c1, Pin(0), 0x3c)

display1.sleep(False)
display1.flip()

display1.fill(0) # clear to black
display1.text('Adler Display', 0, 0, 1) # at x=0, y=0, white on black
display1.hline(0, 9, 127, 1)
display1.hline(0, 30, 127, 1)
display1.vline(0, 10, 32, 1)
display1.vline(127, 10, 32, 1)

for i in range(0, 118):
    display1.fill_rect(i,10, 10, 10, 1)
    display1.fill_rect(10, 21, 30, 8, 0)
    display1.text(str(i), 10, 21, 1)
    display1.show() # update display

from utime import ticks_us, ticks_diff, sleep_ms
from machine import Pin

pin_14 = 14
pin_opto = Pin(pin_14, Pin.IRQ_RISING)

passed = ticks_us()
counter = 0

while True:
    time = ticks_us()
    counter += pin_opto.value()

    if ticks_diff(time, passed) > 100000:                           #100ms interval
        rpm = float(counter * 10 * 60 / ticks_diff(time, passed))
        print("{:.2f}".format(rpm))
        passed = time
        counter = 0

from utime import ticks_us, ticks_diff, sleep_ms
from machine import Pin

pin_14 = 14
pin_opto = Pin(pin_14, Pin.IN)

passed = ticks_us()
counter = 0

def interrupt_handler(pin):
    global counter
    counter += 1

pin_opto.irq(trigger=Pin.IRQ_FALLING, handler=interrupt_handler)

while True:
    sleep_ms(1000)
    time = ticks_us()
    rpm = float(counter / 24 * 52 / 13 / 1 * 60)
    print("{:.2f}".format(rpm))
    counter = 0

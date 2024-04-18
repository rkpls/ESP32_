from utime import ticks_us, ticks_diff, sleep_ms
from machine import Pin, PWM

pin_14 = 14
pin_opto = Pin(pin_14, Pin.IN)

pin_FWD = 12
pwm_fwd = PWM(pin_FWD)
pwm_fwd.duty(0)
pin_RVS = 11
pwm_rvs = PWM(pin_RVS)
pwm_rvs.duty(0)

pwm_fwd.freq(200)
pwm_rvs.freq(200)

counter = 0

def interrupt_handler(pin):
    global counter
    counter += 1

pin_opto.irq(trigger=Pin.IRQ_FALLING, handler=interrupt_handler)

pwm_rvs.duty(0)
pwm_fwd.duty(800)

for i in range (10):
    sleep_ms(200)
    time = ticks_us()
    rpm = float(counter / 24 * 52 / 13 / 1 * 60)
    print("{:.2f}".format(rpm))
    counter = 0

pwm_rvs.duty(0)
pwm_fwd.duty(0)

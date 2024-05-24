import time
from machine import Pin, PWM

pin_FWD = 12
pwm_fwd = PWM(pin_FWD)
pwm_fwd.duty(0)
pin_RVS = 11
pwm_rvs = PWM(pin_RVS)
pwm_rvs.duty(0)


pwm_fwd.freq(200)
pwm_rvs.freq(200)

time.sleep(1)

pwm_rvs.duty(0)
pwm_fwd.duty(400)

time.sleep(1)

pwm_rvs.duty(0)
pwm_fwd.duty(0)

time.sleep(1)

pwm_rvs.duty(400)
pwm_fwd.duty(0)

time.sleep(1)

pwm_rvs.duty(0)
pwm_fwd.duty(0)

time.sleep(1)

#import testfahrt2

from time import sleep
from machine import Pin, PWM

pwm_freq = 200

p4 = Pin(4)
pwm4 = PWM(p4)

print("start")
pwm4.duty(800)
sleep(5)
pwm4.duty(0)
print("end")


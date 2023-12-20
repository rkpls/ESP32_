import machine
from machine import Pin, PWM, SoftI2C
import uasyncio as asyncio
from time import sleep_ms
from imu import MPU6050
from vl53l0x import VL53L0X
import sh1106

dist1 = 0
dist2 = 0
x = 0
y = 0
z = 0
Xstr = 0
Ystr = 0
Zstr = 0

ledxpos = PWM(Pin(42))
ledxneg = PWM(Pin(41))
ledxpos.freq(500)
ledxneg.freq(500)
ledypos = PWM(Pin(40))
ledyneg = PWM(Pin(39))
ledypos.freq(500)
ledyneg.freq(500)
ledzpos = PWM(Pin(38))
ledzneg = PWM(Pin(37))
ledzpos.freq(500)
ledzneg.freq(500)

pin_SDA1 = 15
pin_SCL1 = 16
pin_SDA2 = 17
pin_SCL2 = 18
pin_SDA0 = 8
pin_SCL = 9

loop = asyncio.get_event_loop()

i2c0 = SoftI2C(scl=Pin(pin_SCL), sda=Pin(pin_SDA0), freq=100000)
i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)

imu = MPU6050(i2c0)
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

def led_x():
    i = int(x*255)
    if i >= 0 and i <= 1023:
        ledxpos.duty(i)
    if i <= 0 and i >= -1023:
        i = i *-1
        ledxneg.duty(i)
def led_y():
    i = int(y*255)
    if i >= 0 and i <= 1023:
        ledypos.duty(i)
    if i <= 0 and i >= -1023:
        i = i *-1
        ledyneg.duty(i)
def led_z():
    i = int(z*255)
    if i >= 0 and i <= 1023:
        ledzpos.duty(i)
    if i <= 0 and i >= -1023:
        i = i *-1
        ledzneg.duty(i)
        
          
async def task_accel():
    while True:
        global x, y, z
        x = float(imu.accel.x)
        y = float(imu.accel.y)
        z = float(imu.accel.z)
async def task_led():    
    while True:
        led_x()
        led_y()
        led_z()
async def task_disp():
    while True:
        Xstr = str("X: %0.2f " % x)
        Ystr = str("Y: %0.2f " % y)
        Zstr = str("Z: %0.2f " % z)
        dist1 = str("Sensor 1: %0.0f " % tof1.read())
        dist2 = str("Sensor 2: %0.0f " % tof2.read())
        display1.fill(0)
        display1.text(dist1, 8, 4, 1)
        display1.text(dist2, 8, 20, 1)
        display1.show()
        display2.fill(0)
        display2.text(Xstr, 8, 4, 1)
        display2.text(Ystr, 8, 20, 1)
        display2.text(Zstr, 8, 36, 1)
        display2.show()
        
try:
    loop.create_task(task_accel())             #start loop web
    loop.create_task(task_led())            #start loop data
    loop.create_task(task_disp())               #start loop main
    loop.run_forever()
except Exception as e:
    print("Error Asyncio:", e)
finally:
    loop.close()
    print("close")
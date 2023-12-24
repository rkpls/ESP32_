
from machine import Pin, SoftI2C
from time import sleep_ms
from vl53l0x import VL53L0X


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


i2c3 = SoftI2C(scl=Pin(pin_SCL3), sda=Pin(pin_SDA3), freq=100000)
i2c4 = SoftI2C(scl=Pin(pin_SCL4), sda=Pin(pin_SDA4), freq=100000)

print('Scan i2c bus...')

devices3 = i2c3.scan()
devices4 = i2c4.scan()
   
    
if len(devices3) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:',len(devices3))

  for device3 in devices3:  
    print("Decimal address: ",device3," | Hexa address: ",hex(device3))

if len(devices4) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:',len(devices4))

  for device4 in devices4:  
    print("Decimal address: ",device4," | Hexa address: ",hex(device4))
    
    

# Create VL53L0X objects
tof1 = VL53L0X(i2c3)

# the measuring_timing_budget is a value in ms, the longer the budget, the more accurate the reading.
budget = tof1.measurement_timing_budget_us
tof1.set_measurement_timing_budget(10000)

# Sets the VCSEL (vertical cavity surface emitting laser) pulse period for the
# given period type (VL53L0X::VcselPeriodPreRange or VL53L0X::VcselPeriodFinalRange) 
# to the given value (in PCLKs). Longer periods increase the potential range of the sensor. 
# Valid values are (even numbers only):

# tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 18)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[0], 12)

# tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 14)
tof1.set_Vcsel_pulse_period(tof1.vcsel_period_type[1], 8)


while True:
    print("Tof1: ",tof1.read())

    sleep_ms(100)
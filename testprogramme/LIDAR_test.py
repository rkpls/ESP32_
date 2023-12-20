
from machine import Pin, SoftI2C
from time import sleep_ms
from vl53l0x import VL53L0X


pin_SDA1 = 15
pin_SCL1 = 16
pin_SDA2 = 17
pin_SCL2 = 18
#pin_SDA0 = 8
#pin_SCL = 9

#i2c0 = SoftI2C(scl=Pin(pin_SCL), sda=Pin(pin_SDA0), freq=100000)
i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)

print('Scan i2c bus...')
#devices0 = i2c0.scan()
devices1 = i2c1.scan()
devices2 = i2c2.scan()
   
    
if len(devices1) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:',len(devices1))

  for device1 in devices1:  
    print("Decimal address: ",device1," | Hexa address: ",hex(device1))

if len(devices2) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:',len(devices2))

  for device2 in devices2:  
    print("Decimal address: ",device2," | Hexa address: ",hex(device2))
    
    

# Create VL53L0X objects
tof1 = VL53L0X(i2c1)
tof2 = VL53L0X(i2c2)

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


budget = tof2.measurement_timing_budget_us
tof2.set_measurement_timing_budget(10000)
tof2.set_Vcsel_pulse_period(tof2.vcsel_period_type[0], 12)
tof2.set_Vcsel_pulse_period(tof2.vcsel_period_type[1], 8)



while True:
    print("Tof1: ",tof1.read())
    print("Tof2: ",tof2.read())
    sleep_ms(100)
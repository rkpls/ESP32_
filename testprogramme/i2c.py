
from machine import Pin, SoftI2C

pin_SDA1 = 15
pin_SCL1 = 16
pin_SDA2 = 17
pin_SCL2 = 18
pin_SDA0 = 8
pin_SCL = 9

i2c0 = SoftI2C(scl=Pin(pin_SCL), sda=Pin(pin_SDA0), freq=100000)
i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)

print('Scan i2c bus...')
devices0 = i2c0.scan()
devices1 = i2c1.scan()
devices2 = i2c2.scan()

if len(devices0) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:',len(devices0))

  for device0 in devices0:  
    print("Decimal address: ",device0," | Hexa address: ",hex(device0))
    
    
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
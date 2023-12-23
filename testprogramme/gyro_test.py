
from machine import Pin, SoftI2C
from time import sleep_ms
from imu import MPU6050



pin_SDA5 = 8
pin_SCL5 = 9

i2c5 = SoftI2C(scl=Pin(pin_SCL5), sda=Pin(pin_SDA5), freq=100000)


print('Scan i2c bus...')
devices5 = i2c5.scan()
#devices1 = i2c1.scan()
#devices2 = i2c2.scan()
    
if len(devices5) == 0:
  print("No i2c device !")
else:
  print('i2c devices found:',len(devices5))

  for device5 in devices5:  
    print("Decimal address: ",device5," | Hex address: ",hex(device5))
    
imu = MPU6050(i2c5)
while True:
    sleep_ms(50)
    print("X: %0.1f " % imu.accel.x , "Y: %0.1f " % imu.accel.y, "Z: %0.1f " % imu.accel.z)

from machine import Pin, SoftI2C
from time import sleep_ms
import sh1106


pin_SDA1 = 15
pin_SCL1 = 16
pin_SDA2 = 17
pin_SCL2 = 18
#pin_SDA0 = 8
#pin_SCL = 9

#i2c0 = SoftI2C(scl=Pin(pin_SCL), sda=Pin(pin_SDA0), freq=100000)
i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)
"""
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

"""


display1 = sh1106.SH1106_I2C(128, 64, i2c1, Pin(0), 0x3c)					#keine ahnung wofür pin(0)
oled = sh1106.SH1106_I2C(128, 64, i2c2, Pin(0), 0x3c)

display1.sleep(False)

display1.fill(0)
display1.text('Micropython', 0, 0, 1)
display1.text('Display 1', 0, 16, 1)
display1.show()

width=128
height=64
oled.fill(0) # clear to black

# note that OLEDs have problems with screen burn it - don't leave this on too long!
def border(width, height):
    oled.hline(0, 0, width - 1, 1) # top edge
    oled.hline(0, height - 2, width - 1, 1) # bottom edge
    oled.vline(0, 0, height - 1, 1) # left edge
    oled.vline(width - 1, 0, height - 1, 1) # right edge

# ok, not really a circle - just a square for now
def draw_ball(x,y, size, state):
    if size == 1:
        oled.pixel(x, y, state) # draw a single pixel
    else:
        for i in range(0,size): # draw a box of pixels of the right size
            for j in range(0,size):
                oled.pixel(x + i, y + j, state)
    # TODO: for size above 4 round the corners

border(width, height)

ball_size = 5
current_x = int(width / 2)
current_y = int(height / 2)
direction_x = 1
direction_y = -1
# delay_time = .0001

# oled.line(0, height-2, width-1, height-2, 1)

# Bounce forever
while True:
    draw_ball(current_x,current_y, ball_size,1)
    oled.show()
    # utime.sleep(delay_time)
    draw_ball(current_x,current_y,ball_size,0)
    # reverse at the edges
    # left edge test
    if current_x < 2:
        direction_x = 1
    # right edge test
    if current_x > width - ball_size -2:
        direction_x = -1
    # top edge test
    if current_y < 2:
        direction_y = 1
    # bottom edge test
    if current_y > height - ball_size - 3:
        direction_y = -1
    # update the ball
    current_x = current_x + direction_x
    current_y = current_y + direction_y

print('done')

from machine import Pin, SoftI2C
import machine
from time import sleep_ms
import utime
import sh1106

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

width=128
height=64

i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)

display2 = sh1106.SH1106_I2C(128, 64, i2c2, Pin(0), 0x3c)

display2.sleep(False)
display2.flip()

display2.fill(0) # clear to black
display2.text('Adler Display', 0, 0, 1) # at x=0, y=0, white on black
# line under title
display2.hline(0, 9, 127, 1)
# bottom of display
display2.hline(0, 30, 127, 1)
# left edge
display2.vline(0, 10, 32, 1)
# right edge
display2.vline(127, 10, 32, 1)

for i in range(0, 118):
    # box x0, y0, width, height, on
    display2.fill_rect(i,10, 10, 10, 1)
    # draw black behind number
    display2.fill_rect(10, 21, 30, 8, 0)
    display2.text(str(i), 10, 21, 1)
    display2.show() # update display
    # utime.sleep(0.001)

print('done')


width=128
height=64 # we could make this be 63 but the init method should use the full value
# oled = SSD1306_I2C(width, height, i2c)
oled = sh1106.SH1106_I2C(width, height, i2c1, machine.Pin(4), 0x3c)
oled.flip()
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
# ---------- LIBS ----------
import gc
from machine import Pin, PWM, SPI, SoftI2C
from utime import sleep_ms
import uasyncio as asyncio
import network 

from utime import ticks_us, ticks_diff, sleep_ms 

passed = ticks_us()
counter = 0
from PID import PID
from imu import MPU6050
from vl53l0x import VL53L0X
import sh1106

# ---------- CHANGABLE VARS ----------
dist_front = 0
dist_top = 0
x_gees = 0
y_gees = 0
z_gees = 0

oled1 = 0
oled2 = 0
imu = 0
tof_top = 0
tof_front = 0

pid = PID(1, 0.1, 0.05, setpoint=1, scale='us')

ssid = 'Zenbook-14-Pals'
password = 'Micropython'

# ---------- DATA ----------
volts = float(0.00)                         #
amps = float(0.00)                          #
watts = float(0.00)                         #
g_force_x = int(0)                          #convert to 0-1000 mm/s^2 = 1 m/s^2
g_force_y = int(0)                          # ""
g_force_z = int(0)                          # ""
v_calc = float(0.00)                        # geschwindigkeit mit sensor
v_wheel = float(0.00)                       # geschwindigkeit aus drehzahl
Motor_RPM = float(0.00)                     #

# ---------- VARS ----------
#pwm_freq = 1000                             #
#desired_motor_speed = 0                     #value for manual or calculated input (RPM)
#output = 0                                  #value useed by the PID controller
#current_motor_speed = 0                     #value for sensor reading (RPM)

# ---------- PINS ----------
# --------------------------

Volt_Pin = Pin(4)
Amps_Pin = Pin(5)

pin_SDA2 = 6
pin_SCL2 = 7
pin_SDA3 = 15                                            # RTS 0 UART (Request to send) indicating to the receiver that it should be prepared to receive data.
pin_SCL3 = 16                                            # CTS 0 UART (Clear to Send) indicating to the sender that it can proceed with transmitting data.
pin_SDA4 = 17                                            # TXD 1 UART (Transmit Data) sensor readings, commands, or any other information that needs to be transmitted.
pin_SCL4 = 18                                            # RXD 1 UART (Recieve Data) incoming data,such as commands, sensor readings, or any other information sent by the external device
                                                         # UART = Universal Asynchronous Receiver-Transmitter
pin_SDA5 = 8                                             # !!!! I2C DATA BUS SDA !! used for Gyro 
#Pin_JTAG = 3                   # dont use (JTAG)
#pin_LOG = 46                   # dont use (LOG)
pin_SCL5 = 9                                             # !!!! I2C DATA BUS SCL !! used for Gyro SCK/SCL/SCLK

#pin_CS = 10                                              # !! SS Pin (CS/SS Pin on slave)
#pin_MOSI = 11                                            # !! Mosi Pin (MOSI/SDI on slave) (Master OUT Slave IN)
#pin_MISO = 12                                            # !! Miso Pin (MISO/SDO on slave) (Master IN Slave OUT) 
#pin_SCK = 13                                             # !! SCK Pin (SCK/SCL/SCLK Pin on slave)

pin_14 = 14
pin_opto = Pin(pin_14, Pin.IRQ_RISING)

#----------
#Pin_RX = 43				#dont use
#Pin_TX = 44				#dont use
pin_SDA1 = 1
pin_SCL1 = 2
trg_pin_42 = 42                                                # MTMS (Master Test Mode Select) JTAG (Joint Test Action Group) Debugging and overwriting internal registry
echo_pin_41 = 41                                                # MTDI (Master Test Data Input) JTAG
#pin_40 = 40                                                # MTDO (Master Test Data Output) JTAG
#pin_39 = 39                                                # MTCK (Master Test Clock Signal) JTAG
#pin_builtin_LED = Pin(38, Pin.OUT)
#pin_37 = 37                                                # USB OTG
#pin_36 = 36                                                # USB OTG
#pin_35 = 35                                                # USB OTG
#pins: BOOT / VSPI / RGB_LED
#pin_47 = 47
#pin_21 = 21
#PINS USB_1 / USB_2

#display:
#           SCK is the serial clock signal which will provide the pulses that move the individual bits of data.
#           CS is the chip select signal which allows us to tell the Display when we are talking to it.
#           DC is the Data / Command pin which we use to tell the Display if we are sending it a command instruction or actual data.
#           RESET allows to send a hardware reset signal to the panel.

# ---------- COMMS AND BUS SYSTEM SETUP ----------

# ----------- I2C Sensor Setup + addresses ----------
def i2c_SPI_setup():
    try:
        global i2c1, i2c2, i2c3, i2c4, i2c5
        i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
        i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)
        i2c3 = SoftI2C(scl=Pin(pin_SCL3), sda=Pin(pin_SDA3), freq=100000)
        i2c4 = SoftI2C(scl=Pin(pin_SCL4), sda=Pin(pin_SDA4), freq=100000)
        i2c5 = SoftI2C(scl=Pin(pin_SCL5), sda=Pin(pin_SDA5), freq=100000)
    except:
        print("could not innit i2c")

def sensor_setup():
    try:
        global oled1, oled2, imu, tof_top, tof_front
        
        display1 = sh1106.SH1106_I2C(128, 64, i2c1, Pin(0), 0x3c)
        display2 = sh1106.SH1106_I2C(128, 64, i2c2, Pin(0), 0x3c)
        
        imu = MPU6050(i2c5)
        
        tof_top = VL53L0X(i2c3)
        tof_front = VL53L0X(i2c4)
        tof_top.set_measurement_timing_budget(10000)
        tof_top.set_Vcsel_pulse_period(tof_top.vcsel_period_type[0], 12)
        tof_top.set_Vcsel_pulse_period(tof_top.vcsel_period_type[1], 8)
        tof_front.set_measurement_timing_budget(10000)
        tof_front.set_Vcsel_pulse_period(tof_front.vcsel_period_type[0], 12)
        tof_front.set_Vcsel_pulse_period(tof_front.vcsel_period_type[1], 8)
        
    except:
        print("could not innit sensors or display")
        
# ---------- INIT ----------
i2c_SPI_setup()
sensor_setup()


oled1.fill(0)
oled2.fill(0)
oled1.show()
oled2.show()

# ---------- LOOP ----------

while True:

    dist1 = str(tof_top.read())
    dist2 = str(tof_front.read())
    time = ticks_us()
    counter += pin_opto.value()
    if ticks_diff(time, passed) > 10000:                           #10ms interval
        rpm = float(counter * 100 * 60 / ticks_diff(time, passed))
        passed = time
        counter = 0

    v = controlled_system.update(0)

    control = pid(rpm) 
    v = controlled_system.update(control)

    sleep_ms(50)
    display1.fill(0)
    display1.text(rpm, 0, 0, 1)
    display1.text(v, 0, 16, 1)
    display1.text(" ", 0, 32, 1)
    display1.show()
    display2.fill(0)
    display2.text(dist1, 0, 0, 1)
    display2.text(dist2, 0, 16, 1)
    display2.text("", 0, 32, 1)
    display2.show()
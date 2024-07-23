# ---------- LIBS ----------
import gc
from machine import Pin, PWM, ADC, SoftI2C
from utime import ticks_us, ticks_diff, sleep_ms 
import uasyncio as asyncio
import network

from imu import MPU6050
from vl53l0x import VL53L0X
import sh1106

loop = asyncio.get_event_loop()
# ---------- CHANGABLE VARS ----------

ssid = '***'
password = '***'
wlan = network.WLAN(network.STA_IF)

system_check = 10

dist_front = 8190
dist_top = 8190
x_gees = 0
y_gees = 0
z_gees = 0

dist_top_values = []
dist_front_values = []

oled1 = 0
oled2 = 0
imu = 0
tof_top = 0
tof_front = 0
tof_top_active = False
tof_front_active = False

rpm = 0
passed = ticks_us()
counter = 0

pwm_freq = 200

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

Volt_Pin = ADC(Pin(4))                      #0.2V per V
Amps_Pin = ADC(Pin(5))                      #0.1V per A

pin_SDA2 = 6
pin_SCL2 = 7
pin_SDA3 = 15                                             # RTS 0 UART (Request to send) indicating to the receiver that it should be prepared to receive data.
pin_SCL3 = 16                                             # CTS 0 UART (Clear to Send) indicating to the sender that it can proceed with transmitting data.
pin_SDA4 = 17                                             # TXD 1 UART (Transmit Data) sensor readings, commands, or any other information that needs to be transmitted.
pin_SCL4 = 18                                             # RXD 1 UART (Recieve Data) incoming data,such as commands, sensor readings, or any other information sent by the external device
                                                          # UART = Universal Asynchronous Receiver-Transmitter
pin_SDA5 = 8                                              # !!!! I2C DATA BUS SDA !! used for Gyro 
#Pin_JTAG = 3                   # dont use (JTAG)
#pin_LOG = 46                   # dont use (LOG)
pin_SCL5 = 9                                              # !!!! I2C DATA BUS SCL !! used for Gyro SCK/SCL/SCLK

#pin_CS = 10                                              # !! SS Pin (CS/SS Pin on slave)
pin_FWD = 11                                              # !! Mosi Pin (MOSI/SDI on slave) (Master OUT Slave IN)
pwm_fwd = PWM(pin_FWD)
pin_RVS = 12                                              # !! Miso Pin (MISO/SDO on slave) (Master IN Slave OUT) 
pwm_rvs = PWM(pin_RVS)
#pin_SCK = 13                                             # !! SCK Pin (SCK/SCL/SCLK Pin on slave)
rpm_pin_14 = Pin(14, Pin.IN)
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


# ---------- COMMS SETUP ----------
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    return wlan

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
        system_check -1
        print("could not innit i2c")

def sensor_setup():
    global oled1, oled2, imu, tof_top, tof_front, tof_top_active, tof_front_active
    try:    
        oled1 = sh1106.SH1106_I2C(128, 64, i2c1, Pin(0), 0x3c)                              #rechts
        oled2 = sh1106.SH1106_I2C(128, 64, i2c2, Pin(0), 0x3c)                              #links
    except:
        print("could not innit displays")
    try:    
        imu = MPU6050(i2c5)
    except:
        print("could not innit imu")
    try:
        tof_top = VL53L0X(i2c3)
        tof_top.set_measurement_timing_budget(10000)
        tof_top.set_Vcsel_pulse_period(tof_top.vcsel_period_type[0], 12)
        tof_top.set_Vcsel_pulse_period(tof_top.vcsel_period_type[1], 8)
        tof_top_active = True
    except:
        tof_top_active = False
        print("could not innit tof top sensor")
    try:
        tof_front = VL53L0X(i2c4)
        tof_front.set_measurement_timing_budget(10000)
        tof_front.set_Vcsel_pulse_period(tof_front.vcsel_period_type[0], 12)
        tof_front.set_Vcsel_pulse_period(tof_front.vcsel_period_type[1], 8)
        tof_front_active = True
    except:
        tof_front_active = False
        print("could not innit tof front sensor")        

def average(values):
    if len(values) > 0:
        v = sum(values) / len(values)
        return round(v,1)
    else:
        return 0
        
def read_dist_sensors():
    global dist_top, dist_front, dist_top_values, dist_front_values
    if tof_top_active:
        try:
            dist_top_values.append(int(tof_top.read()))
            if len(dist_top_values) >= 5:
                dist_top_values.pop(0)
            dist_top = average(dist_top_values)
        except:
            system_check -1
    if tof_front_active:
        try:
            dist_front_values.append(int(tof_front.read()))
            if len(dist_front_values) >= 5:
                dist_front_values.pop(0)
            dist_front = average(dist_front_values)
        except:
            system_check -1

def read_accel():
    global x_gees, y_gees, z_gees
    try:
        x_gees = float(imu.accel.x)
        y_gees = float(imu.accel.y)
        z_gees = float(imu.accel.z)
    except:
        pass
        
def read_rpm():
    global rpm, passed, counter
    time = ticks_us()
    if (ticks_diff(time, passed) > 500):
        passed = time   
        time = ticks_us()
        rpm = float(counter / 24 * 52 / 13 * 60)
        counter = 0
    else:
        pass

def interrupt_handler(pin):
    global counter
    counter += 1

def read_batt():
    global volts, amps
    try:
        v_read = int(Volt_Pin.read())
        volts = v_read * 5 / (4069/3.3)
        a_read = int(Volt_Pin.read() )
        amps = a_read / 10 / (4069/3.3)
    except:
        print("could not read battery status")
        
def refresh_oled():
    while True:
        oled1.fill(0)
        oled1.text("Adler Sensoren", 0, 0, 1)
        oled1.text("Oben:", 0, 16, 1)
        oled1.text("Vorne:", 0, 32, 1)
        oled1.text("Beschl.:", 0, 48, 1)
        data_t = str(dist_top)
        oled1.text(data_t, 96, 16, 1)
        data_f = str(dist_front)
        oled1.text(data_f, 96, 32, 1)
        data_g = str(x_gees)
        oled1.text(data_g, 96, 48, 1)
        oled1.show()
        
        oled2.fill(0)
        oled2.text("Adler Ueberwachung", 0, 0, 1)
        oled2.text("Spannung:", 0, 16, 1)
        oled2.text("Strom:", 0, 32, 1)
        oled2.text("RPM:", 0, 48, 1) 
        data_v = str(volts)
        oled2.text(data_v, 80, 16, 1)
        data_a = str(amps)
        oled2.text(data_a, 80, 32, 1)
        data_rpm = str(rpm)
        oled2.text(data_rpm, 80, 48, 1)    
        oled2.show()
    
def motor_control():
    if system_check == 10:
        if dist_top < 120 and dist_top > 50:
            pwm_fwd.duty(700)
        else:
            pwm_fwd.duty(0)
    else:
        pwm_fwd.duty(0)
        print("sensor failure")
# ---------- INIT ----------
pwm_fwd.duty(0)


Volt_Pin.atten(ADC.ATTN_11DB)
Amps_Pin.atten(ADC.ATTN_11DB)
try:
    oled1.flip()
    oled2.flip()
    oled1.fill(0)
    oled2.fill(0)
    oled1.show()
    oled2.show()
except:
    pass

rpm_pin_14.irq(trigger=Pin.IRQ_FALLING, handler=interrupt_handler)

# ---------- ASYNC DEF ----------

async def sensors():
    i2c_SPI_setup()
    sensor_setup()
    while True:
        read_dist_sensors()
        read_accel()
        read_rpm()
        read_batt()

async def monitor():
    while True:
        refresh_oled()
        

# ---------- SETUP ----------
print("MAIN")
if wlan.isconnected():
    print(ip)
    gc.enable()




# ---------- LOOP ----------

    
    motor_control()

try:
    loop.create_task(task_socket())
    loop.create_task(task_main())
    loop.create_task(task_sensors())
    loop.run_forever()
except Exception as e:
    print("Error:", e)
finally:
    pwm16.duty(0)
    pwm17.duty(0)
    loop.close()
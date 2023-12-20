

# ---------- LIBS ----------
import gc
from machine import Pin, PWM, SPI, SoftI2C, time_pulse_us            
from utime import sleep_us
import uasyncio as asyncio                  #multitasking
import gc                                   #RAM manager
import network                              #wifi module
import hcsr04 as HCSR04                     #distance sensor ----- install hcsr04.py
import mpu6050  as MPU6050                  #G-Force Sensor ----- install GY521.py and vectro3d.py
import esp2in9bv2 as esp2in9bv2
from esp2in9bv2 import Display


# ---------- CHANGABLE VARS ----------
wlan = 0
ssid = "Zenfone-9"
password = "Micropython"

# ---------- DATA ----------
volts = float(0.00)                         #
amps = float(0.00)                          #
watts = float(0.00)                         #
Motor_RPM = float(0.00)                     #
g_force_x = int(0)                          #convert to 0-1000 mm/s^2 = 1 m/s^2
g_force_y = int(0)                          # ""
g_force_z = int(0)                          # ""

# ---------- VARS ----------
pwm_freq = 1000                             #
desired_motor_speed = 0                     #value for manual or calculated input (RPM)
output = 0                                  #value useed by the PID controller
current_motor_speed = 0                     #value for sensor reading (RPM)

#

# ---------- PINS ----------
# --------------------------
Motor_Pin_1 = Pin(4)
motor_pwm_1 = PWM(Motor_Pin_1)
Motor_Pin_2 = Pin(5)
motor_pwm_2 = PWM(Motor_Pin_2)

pin_SDA1 = 15                                             # RTS 0 UART (Requiest to send) indicating to the receiver that it should be prepared to receive data.
pin_SCL1 = 16                                             # CTS 0 UART (Clear to Send) indicating to the sender that it can proceed with transmitting data.
pin_SDA2 = 17                                             # TXD 1 UART (Transmit Data) sensor readings, commands, or any other information that needs to be transmitted.
pin_SCL2 = 18                                             # RXD 1 UART (Recieve Data) incoming data,such as commands, sensor readings, or any other information sent by the external device
                                                         # UART = Universal Asynchronous Receiver-Transmitter
pin_SDA0 = 8                                             # !!!! I2C DATA BUS SDA !! used for Gyro 
#Pin_JTAG = 3                   # dont use (JTAG)
#pin_LOG = 46                   # dont use (LOG)
pin_SCL = 9                                             # !!!! I2C DATA BUS SCL !! used for Gyro SCK/SCL/SCLK

pin_CS = 10                                              # !! SS Pin (CS/SS Pin on slave)
pin_MOSI = 11                                            # !! Mosi Pin (MOSI/SDI on slave) (Master OUT Slave IN)
pin_MISO = 12                                            # !! Miso Pin (MISO/SDO on slave) (Master IN Slave OUT) 
pin_SCK = 13                                             # !! SCK Pin (SCK/SCL/SCLK Pin on slave)
                                                         # SPI = Serial Peripheral Interface bus protocol
pin_DC = 14

Volts_Pin = Pin(43, Pin.IN)              #  TXD 0 UART // Volts 1 Volts = 200mV (max read: 3.3V->16,5V -> Bat 12V/5)
Amps_Pin = Pin(44, Pin.IN)               #  TXD 0 UART // Amps 1 Amps = 100mV

pin_1 = 1
pin_2 = 2
#pin_42 = 42                                                # MTMS (Master Test Mode Select) JTAG (Joint Test Action Group) Debugging and overwriting internal registry
#pin_41 = 41                                                # MTDI (Master Test Data Input) JTAG
#pin_40 = 40                                                # MTDO (Master Test Data Output) JTAG
#pin_39 = 39                                                # MTCK (Master Test Clock Signal) JTAG
#pin_builtin_LED = machine.Pin(38, machine.Pin.OUT)
#pin_37 = 37                                                # USB OTG
#pin_36 = 36                                                # USB OTG
#pin_35 = 35                                                # USB OTG
#pins: BOOT / VSPI / RGB_LED
pin_rst = 47
pin_busy = 21
#PINS USB_1 / USB_2

#display:
#           SCK is the serial clock signal which will provide the pulses that move the individual bits of data.
#           CS is the chip select signal which allows us to tell the Display when we are talking to it.
#           DC is the Data / Command pin which we use to tell the Display if we are sending it a command instruction or actual data.
#           RESET allows to send a hardware reset signal to the panel.

# ---------- COMMS AND BUS SYSTEM SETUP ----------
loop = asyncio.get_event_loop()

try:
    spi = SPI(0, baudrate=4000000, sck=Pin(pin_SCK), mosi=Pin(pin_MOSI), miso=Pin(pin_MISO))         # !!! SPI !!!
    display = Display(spi, dc=Pin(pin_DC), cs=Pin(pin_CS), rst=Pin(pin_rst), busy=Pin(pin_busy))   # e-paper
except:
    print("no SPI")

i2c0 = SoftI2C(scl=Pin(pin_SCL), sda=Pin(pin_SDA0), freq=100000)
i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)

mpu6050 = MPU6050(i2c0)                                                                          # gyro


# ---------- FUNCTIONS ----------
def wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)                     #setup wifi mode
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting')
        try:
            for i in range (5):
                wlan.connect(ssid, password)
                time.sleep(1)
        except:
            pass

def gyro():
    try:
        global g_force_x, g_force_y, g_force_z
        read_x = float(MPU6050.gyro.x)
        g_force_x = read_x / 4069
        read_y = float(MPU6050.gyro.y)
        g_force_y = read_y/ 4069    
        read_z = float(MPU6050.gyro.z)
        g_force_z = read_z /4096
        print(g_force_x, g_force_y, g_force_z)
    except:
        print("Could not read accelerometer")

def distance_meaasurement(self):
    self.trigger.value(0)
    sleep_us(5)
    self.trigger.value(1)
    sleep_us(10)
    
    dist = pulse_time = time_pulse_us(self.echo, 1, self.echo_timeout_us)
    return dist

def test_display():
    print('Done.')

def display_qr():
    pass

def read_desired_motor_speed():
    global desired_motor_speed
    try:
        pass
    except:
        print("could not resolve desired motor speed")

def read_current_motor_spped():
    global current_motor_speed
    try:
        pass                                                                #add rpm sensor reading here
    except:
        print("could not read motor speed")
            
            
class PIDController:
    def __init__(self, setpoint, Kp, Ki, Kd):                               #PID class setup for update function
        self.setpoint = setpoint
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def update(self, current_process_value):
        error = self.setpoint - current_process_value
        proportional_term = self.Kp * error                                     # Calculate the proportional term
        self.integral += error
        integral_term = self.Ki * self.integral                                 # Calculate the integral term
        derivative_term = self.Kd * (error - self.previous_error)               # Calculate the derivative term
        self.previous_error = error                                             # Update the previous error value
        control_output = proportional_term + integral_term + derivative_term    # Calculate the control output
        return control_output
    
def set_motor_speed(power):
    duty = power / 100 * 1023                   # ---- CHANGE THIS NEED BE RPM / power request to in Hz
    if duty > 127:                              #vorwärts min 12.5%
        motor_pwm_1.duty(0)
        motor_pwm_2.freq(pwm_freq)
        duty = int(duty)                        #make integer
        motor_pwm_2.duty(duty)
    elif duty < -127:                           #back min 12.5%
        motor_pwm_2.duty(0)
        motor_pwm_1.freq(pwm_freq)
        duty = int(duty * -1)
        motor_pwm_1.duty(duty)
    else:                                       #motor aus
        motor_pwm_1.duty(0)
        motor_pwm_2.duty(0)

# ---------- TASKS ----------
# ---------- WEB AND COMMS ----------
async def task_socket():
    while True:
        pass


# ---------- DATA MANAGE ----------
async def task_monitor():                       # ---- TESTING! MAY NOT WORK ----
    MPU6050.accel_range(2)                      #max G: +-8
    MPU6050.accel_filter_range(3)               #filters vibrations averages every 11.8ms
    while True:
        gyro()

# ---------- MAIN CONTROL ----------
async def task_main():
    while True:
        #read_desired_motor_speed()
        #read_current_motor_spped()
        #measurement = current_motor_speed()        #Read your measurement here
        #dt = time.time() - last_time            #Calculate the time difference
        #output = pid.update(measurement, dt)
        #set_motor_speed(output)                 #use the calculatet output variable as power in set_motor_speed
        #last_time = time.time()
        pass

# ---------- MULTI ----------
gc.enable()                                     #enable auto RAM Manager
while not wlan.isconnected():
    try:
        wifi(ssid, password)                            #connect to wifi
    except:
        print("could not find a WIFI")

display_qr()

try:
    loop.create_task(task_socket())             #start loop web
    loop.create_task(task_monitor())            #start loop data
    loop.create_task(task_main())               #start loop main
    loop.run_forever()
except Exception as e:
    print("Error Asyncio:", e)
finally:
    motor_pwm_1.duty(0)
    motor_pwm_2.duty(0)
    loop.close()
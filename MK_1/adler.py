

# ---------- LIBS ----------                           
import machine                              #Pins and Stuff
from machine import Pin, I2C                
import time
import asyncio                              #multitasking
from asyncio import loop
import gc                                   #RAM manager
import network                              #wifi module
import hcsr04 as HCSR04                     #distance sensor ----- install hcsr04.py
import mpu6050 as MPU6050                   #G-Force Sensor ----- install mpu6050.py and vectro3d.py



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
pid = PIDController(1.0, 0.1, 0.05, 1)      #values for kp, ki, kd, and setpoint


# ---------- PINS ----------
i2c = I2C(scl=Pin(9), sda=Pin(8))                    # !!! I2C !!!
mpu6050 = MPU6050(i2c)
# --------------------------
Motor_Pin_1 = machine.Pin(4)
motor_pwm_1 = machine.PWM(Motor_Pin_1)
Motor_Pin_2 = machine.Pin(5)
motor_pwm_2 = machine.PWM(Motor_Pin_2)

HCSR_sensor_1 = HCSR04(trigger_pin=6, echo_pin=7, echo_timeout_us=10000)
HCSR_sensor_2 = HCSR04(trigger_pin=15, echo_pin=16, echo_timeout_us=10000)

RPM_sensor = machine.Pin(17, machine.Pin.IN)             #optocoupler read

#pin_18 = 18
#pin_SDA = 8                                             # !!! I2C DATA BUS SDA !!! used for Gyro
#Pin_JTAG = 3
#pin_LOG = 46
#pin_SCL = 9                                             # !!! I2C DATA BUS SCL !!! used for Gyro
#pin_SS = 10
#pin_MOSI = 11
#pin_MISO = 12
#pin_SCK = 13
#pin_14 = 14

Volts_Pin = machine.Pin(43, machine.Pin.IN)               #Volts 1 Volts = 200mV (max read: 3.3V->16,5V -> Bat 12V/5)
Amps_Pin = machine.Pin(44, machine.Pin.IN)                #Amps 1 Amps = 100mV

#pin_1 = 1
#pin_2 = 2
#pin_42 = 42
#pin_41 = 41
#pin_40 = 40
#pin_39 = 39
#pin_builtin_LED = machine.Pin(38, machine.Pin.OUT)
#pin_37 = 37
#pin_36 = 36
#pin_35 = 35
#pins: BOOT / VSPI / RGB_LED
#pin_47 = 47
#pin_21 = 21
#PINS USB_1 / USB_2

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
        read_x = float(mpu6050.gyro.x)
        g_force_x = read_x / 4069
        read_y = float(mpu6050.gyro.y)
        g_force_y = read_y/ 4069    
        read_z = float(mpu6050.gyro.z)
        g_force_z = read_z /4096
        print(g_force_x, g_force_y, g_force_z)
    except:
        print("Could not read accelerometer")


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
        motor_pwm_2.duty(duty)
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
    mpu6050.accel_range(2)                      #max G: +-8
    mpu6050.accel_filter_range(3)               #filters vibrations averages every 11.8ms
    while True:
        gyro()


# ---------- MAIN CONTROL ----------
async def task_main():
    while True:
        read_desired_motor_speed()
        read_current_motor_spped()
        measurement = current_motor_speed()        #Read your measurement here
        dt = time.time() - last_time            #Calculate the time difference
        output = pid.update(measurement, dt)
        set_motor_speed(output)                 #use the calculatet output variable as power in set_motor_speed
        last_time = time.time()

# ---------- MULTI ----------
gc.enable()                                     #enable auto RAM Manager
while not wlan.isconnected():
    try:
        wifi(ssid, password)                            #connect to wifi
    except:
        print("could not find a WIFI")

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
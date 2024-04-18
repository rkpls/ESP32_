# ---------- LIBS ----------
import sys
import gc
from machine import Pin, PWM, ADC, SoftI2C, unique_id, reset
from utime import ticks_us, ticks_ms, ticks_diff, sleep_ms
import network
from ubinascii import hexlify
from umqtt.simple import MQTTClient
import json
from imu import MPU6050
from vl53l0x import VL53L0X
import sh1106

# ---------- CHANGABLE VARS ----------

ssid = 'Zenbook-14-Pals'
password = 'Micropython'
wlan = network.WLAN(network.STA_IF)
MQTT_SERVER = '192.168.137.1'
CLIENT_ID = hexlify(unique_id())
MQTT_TOPIC = 'adler/data/*'
MQTT_RECEIVE_TOPIC = 'adler/status/#'

# ---------- DATA ----------
volts = float(0.00)
amps = float(0.00)
g_force_x = int(0)                          #convert to 0-1000 mm/s^2 = 1 m/s^2
g_force_y = int(0)
g_force_z = int(0)
rpm = float(0.00)

# ---------- VARS ----------
dist_top = 0
dist_front = 0
x_gees = 0
y_gees = 0
z_gees = 0
volts = 0
amps = 0
revtime = []
passed = ticks_ms()
status = False
anfahrt_aktiv = False
durchfahrt_aktiv = False
bremsung_aktiv = False
ende = False
pwm_freq = 1000
revcounter = 0
dist_front = 8190
dist_top = 8190
dist_top_values = []
dist_front_values = []
x_gees = 0
y_gees = 0
z_gees = 0

# ---------- PINS ----------
Volt_Pin = ADC(Pin(4))                      #0.2V per V
Amps_Pin = ADC(Pin(5))                      #0.1V per A
pin_SDA2 = 6
pin_SCL2 = 7
pin_SDA3 = 15 
pin_SCL3 = 16 
pin_SDA4 = 17 
pin_SCL4 = 18
pin_SDA5 = 8 
pin_SCL5 = 9 
pin_FWD = 12
pin_RVS = 11
rpm_pin_14 = Pin(14, Pin.IN)
pin_SDA1 = 1
pin_SCL1 = 2

# ----------- BUS ----------
pwm_fwd = PWM(pin_FWD)
pwm_fwd.duty(0)
pwm_rvs = PWM(pin_RVS)
pwm_rvs.duty(0)
pwm_freq = 1000
pwm_fwd.freq(pwm_freq)
pwm_rvs.freq(pwm_freq)
kp = 0.5
ki = 0.1
kd = 0.2
setpoint_rpm = 1000
pid = PIDController(kp, ki, kd, setpoint_rpm)
i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
oled1 = sh1106.SH1106_I2C(128, 64, i2c1, Pin(0), 0x3c)                              #rechts
oled1.flip()
oled1.fill(0)
oled1.show()
i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)
oled2 = sh1106.SH1106_I2C(128, 64, i2c2, Pin(0), 0x3c)                              #links
oled2.flip()
oled2.fill(0)
oled2.show()
i2c3 = SoftI2C(scl=Pin(pin_SCL3), sda=Pin(pin_SDA3), freq=100000)
i2c4 = SoftI2C(scl=Pin(pin_SCL4), sda=Pin(pin_SDA4), freq=100000)
i2c5 = SoftI2C(scl=Pin(pin_SCL5), sda=Pin(pin_SDA5), freq=100000)
tof_top = VL53L0X(i2c3)
tof_top.set_measurement_timing_budget(10000)
tof_top.set_Vcsel_pulse_period(tof_top.vcsel_period_type[0], 12)
tof_top.set_Vcsel_pulse_period(tof_top.vcsel_period_type[1], 8)
tof_front = VL53L0X(i2c4)
tof_front.set_measurement_timing_budget(10000)
tof_front.set_Vcsel_pulse_period(tof_front.vcsel_period_type[0], 12)
tof_front.set_Vcsel_pulse_period(tof_front.vcsel_period_type[1], 8)
imu = MPU6050(i2c5)
Volt_Pin.atten(ADC.ATTN_11DB)
Amps_Pin.atten(ADC.ATTN_11DB)

# ----------- DEFS ----------
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    print(wlan.ifconfig())
    return wlan

def mqtt_callback(topic, msg):
    global status
    print("Received MQTT message:", msg.decode())
    status = True

def average(values):
    if len(values) > 0:
        return sum(values) / len(values)
    else:
        return 1
    
def interrupt_handler():
    global revtime
    now = ticks_us()
    revtime.append(round(ticks_diff(now, last), 4))
    last = now

def interrupt_handler_rpm(pin):
    global revcounter
    revcounter += 1
    
def idle_loop_1sec():
    passed = 0
    time = ticks_ms()
    if (ticks_diff(time, passed) < 1000):
        sensors()
        system_check()
        monitoring_send()
        display1()
        display2()
        passed = time

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, feedback_value):
        error = self.setpoint - feedback_value
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# ---------- THREAD DEF ----------
def system_check():
    global system
    if volts < 8 or amps > 10:
        system = False
    else:
        system = True

def monitoring_send(server = MQTT_SERVER): 
    global passed, CLIENT_ID, MQTT_SERVER, MQTT_TOPIC, x_gees, volts, amps, rpm
    passed = ticks_ms()
    interval = 500
    time = ticks_ms()
    if (ticks_diff(time, passed) > interval):
        try:
            last_logs = sys.stdout.buffer.getvalue().decode().split('\n')
            sys.stdout.buffer.truncate(0)
            sys.stdout.buffer.seek(0)
            client = MQTTClient(CLIENT_ID, MQTT_SERVER)
            client.connect()
            data = {'logs': str(last_logs),
                    'Volt:': str(volts),
                    'Amps': str(amps),
                    'Rpm': str(rpm),
                    'Accelx': str(x_gees),
                    'Accely': str(y_gees),
                    'Accelz': str(z_gees),
                    'DistT': str(dist_top),
                    'DistF': str(dist_front)}
            dump = json.dumps(data)
            client.publish(MQTT_TOPIC, dump)
            client.disconnect()
        except:
            print("connection to mqqt broker failed")
        passed = time
        
def abfahrt():
    global status, target_rpm, anfahrt_aktiv, durchfahrt_aktiv, bremsung_aktiv, ende
    interval = 50
    passed = ticks_ms()
    time = ticks_ms()
    if (ticks_diff(time, passed) > interval):     
        if status == True:
            status = False
            anfahrt_aktiv = True
        if anfahrt_aktiv == True:
            target_rpm = 300
        if anfahrt_aktiv == True and dist_top < 200:
            anfahrt_aktiv = False
            durchfahrt_aktiv = True
        if  durchfahrt_aktiv == True:
            if dist_front > 100:
                target_rpm = 400
            if dist_front <= 100:
                target_rpm = 200
        if durchfahrt_aktiv == True and dist_top > 200:
            durchfahrt_aktiv = False
            bremsung_aktiv = True       
        if  bremsung_aktiv == True:
            target_rpm = 0
        if bremsung_aktiv == True and rpm == 0 and x_gees <= 0.01:
            bremsung_aktiv = False
            ende = True
        passed = time

def sensors():
    global rpm, dist_top, dist_front, x_gees, y_gees, z_gees, volts, amps
    time = ticks_ms()
    interval = 50
    time = ticks_ms()
    if (ticks_diff(time, passed) > interval):
        rpm = round(float(revcounter / 24 * 52 / 13 / 1 * 60),1)
        dist_top_values.append(int(tof_top.read()))
        if len(dist_top_values) > 3:
            dist_top_values.pop(0)
        dist_top = average(dist_top_values)
        dist_front_values.append(int(tof_front.read()))
        if len(dist_front_values) > 3:
            dist_front_values.pop(0)
        dist_front = average(dist_front_values)
        x_gees = float(round(imu.accel.x, 1))
        y_gees = float(round(imu.accel.y, 1))
        z_gees = float(round(imu.accel.z, 1))
        v_read = int(Volt_Pin.read())
        volts = round(v_read * 5 / (4069/3.3), 1)
        a_read = int(Amps_Pin.read() )
        amps = round(a_read / 10 / (4069/3.3), 2)
        passed = time

def motor_control():
    global pwm_output
    passed = ticks_ms()
    time = ticks_ms()
    interval = 50
    if (ticks_diff(time, passed) > interval):
        pwm_output = pid.compute(rpm)
        duty = int(target_rpm * 1)
        pwm_fwd.duty(duty)
        pwm_rvs.duty(0)
        print(duty)
        passed = time

def display1():
    passed = ticks_ms()
    time = ticks_ms()
    interval = 500
    if (ticks_diff(time, passed) > interval) and (anfahrt_aktiv == False or durchfahrt_aktiv == False or bremsung_aktiv == False):
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
        passed = time

def display2():
    passed = ticks_ms()
    time = ticks_ms()
    interval = 500
    if (ticks_diff(time, passed) > interval) and (anfahrt_aktiv == False or durchfahrt_aktiv == False or bremsung_aktiv == False):
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
        passed = time
            
# ---------- STARTUP ----------

connect_wifi()
gc.enable()
rpm_pin_14.irq(trigger=Pin.IRQ_FALLING, handler=interrupt_handler_rpm)
print("warten auf mqtt input an adler/status/#")
sensors()
monitoring_send()
client = MQTTClient(CLIENT_ID, MQTT_SERVER)
client.connect()
client.set_callback(mqtt_callback)
client.subscribe(MQTT_RECEIVE_TOPIC)
client.wait_msg()
print("starte in 5 sekunden")
for i in range (5):
    idle_loop_1sec()

# ---------- LOOP ----------
while True:
    try:
        sensors()
        system_check()
        monitoring_send()
        display1()
        display2()
        if system == True:
            abfahrt()
            motor_control()
        else:
            pwm_fwd.duty(0)
            pwm_rvs.duty(0)
            idle_loop_1sec()
            reset()

    except Exception as e:
        print("Error:", e)
        sleep_ms(500)
        reset()


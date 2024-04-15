# ---------- LIBS ----------
import sys
import gc
from machine import Pin, PWM, ADC, SoftI2C, unique_id, reset
from utime import ticks_us, ticks_ms, ticks_diff, sleep_ms
import uasyncio as asyncio
import network
from ubinascii import hexlify
from umqtt.simple import MQTTClient
import json

from imu import MPU6050
from vl53l0x import VL53L0X
import sh1106

loop = asyncio.get_event_loop()
# ---------- CHANGABLE VARS ----------

ssid = 'Zenbook-14-Pals'
password = 'Micropython'
wlan = network.WLAN(network.STA_IF)
MQTT_SERVER = '192.168.137.1'
CLIENT_ID = hexlify(unique_id())
MQTT_TOPIC = 'adler/data/*'
MQTT_RECEIVE_TOPIC = 'adler/status/#'

# ---------- DATA ----------
volts = float(0.00)                         #
amps = float(0.00)                          #
watts = float(0.00)                         #
g_force_x = int(0)                          #convert to 0-1000 mm/s^2 = 1 m/s^2
g_force_y = int(0)                          # ""
g_force_z = int(0)                          # ""
v_calc = float(0.00)                        # geschwindigkeit mit sensor
v_wheel = float(0.00)                       # geschwindigkeit aus drehzahl
rpm = float(0.00)                     		#

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
durchfahrt_aktiv = False
anfahrt = False

pwm_freq = 1000                             #
target_rpm = 0                              #value for manual or calculated input (RPM)
output = 0                                  #value useed by the PID controller
target_rpm = 0
current_rpm = 0                             #value for sensor reading (RPM)

kp = 0.1                                    #proportional
ki = 0.01                                   #integral
kd = 0.01                                   #derivative
prev_error = 0
integral = 0
base_setpoint = 100
max_setpoint = 400                         # Maximum prm

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
pwm_fwd.duty(0)
pin_RVS = 12                                              # !! Miso Pin (MISO/SDO on slave) (Master IN Slave OUT) 
pwm_rvs = PWM(pin_RVS)
pwm_rvs.duty(0)
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




# ----------- DEFS ----------

def connect_wifi():                                         #fn für WLAN
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
    print(status)

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

# ---------- THREAD DEF ----------

async def monitoring_send(server = MQTT_SERVER): 
    global passed, CLIENT_ID, MQTT_SERVER, MQTT_TOPIC, x_gees, volts, amps, rpm
    passed = ticks_ms()
    interval = 1000
    while True:
        time = ticks_ms()
        if (ticks_diff(time, passed) > interval):
            try:
                last_logs = sys.stdout.buffer.getvalue().decode().split('\n')
                sys.stdout.buffer.truncate(0)
                sys.stdout.buffer.seek(0)
                client = MQTTClient(CLIENT_ID, MQTT_SERVER)
                client.connect()
                data = {
                    'logs': str(last_logs),
                    'Volt:': str(volts),
                    'Amps': str(amps),
                    'Rpm': str(rpm),
                    'Accel': str(x_gees)
                    }
                dump = json.dumps(data)
                client.publish(MQTT_TOPIC, dump)
                client.disconnect()
            except:
                pass
            
            passed = time
        await asyncio.sleep_ms(500)

async def abfahrt():
    global status, target_rpm, anfahrt, durchfahrt_aktiv
    interval = 100
    passed = ticks_ms()
    while True:
        time = ticks_ms()
        if (ticks_diff(time, passed) > interval):
            
            if status == True:
                anfahrt = True
                target_rpm = 300
                print("Anfahrt")
            else:
                print("ende")
                target_rpm = 0
                reset()
                
            if dist_top <= 400 and anfahrt == True:
                status = False
                anfahrt = False
                if dist_front > 200:
                    target_rpm = 400
                    print("set 400")
                if dist_front <= 200:
                    target_rpm = 200
                    print("set 200")
                durchfahrt_aktiv = True
                    
            if dist_top > 400 and durchfahrt_aktiv == True:
                durchfahrt_aktiv = False
                pwm_fwd.duty(0)
                pwm_rvs.duty(0)
                target_rpm = 0
                print("Bremsung")
        
            passed = time
        await asyncio.sleep_ms(200)

async def sensors():
    global dist_top, dist_front, x_gees, y_gees, z_gees, volts, amps, rpm
    dist_front = 8190
    dist_top = 8190
    dist_top_values = []
    dist_front_values = []
    x_gees = 0
    y_gees = 0
    z_gees = 0
    
    i2c3 = SoftI2C(scl=Pin(pin_SCL3), sda=Pin(pin_SDA3), freq=100000)
    i2c4 = SoftI2C(scl=Pin(pin_SCL4), sda=Pin(pin_SDA4), freq=100000)
    i2c5 = SoftI2C(scl=Pin(pin_SCL5), sda=Pin(pin_SDA5), freq=100000)

    try:
        tof_top = VL53L0X(i2c3)
        tof_top.set_measurement_timing_budget(10000)
        tof_top.set_Vcsel_pulse_period(tof_top.vcsel_period_type[0], 12)
        tof_top.set_Vcsel_pulse_period(tof_top.vcsel_period_type[1], 8)
    except:
        print("could not innit tof top sensor")
    try:
        tof_front = VL53L0X(i2c4)
        tof_front.set_measurement_timing_budget(10000)
        tof_front.set_Vcsel_pulse_period(tof_front.vcsel_period_type[0], 12)
        tof_front.set_Vcsel_pulse_period(tof_front.vcsel_period_type[1], 8)
    except:
        print("could not innit tof front sensor")
    try:
        imu = MPU6050(i2c5)
    except:
        print("could not innit imu")        
    Volt_Pin.atten(ADC.ATTN_11DB)
    Amps_Pin.atten(ADC.ATTN_11DB)
    
    passed = ticks_ms()
    while True:
        interval = 100
        time = ticks_ms()
        if (ticks_diff(time, passed) > interval):
            try:
                dist_top_values.append(int(tof_top.read()))
                if len(dist_top_values) >= 5:
                    dist_top_values.pop(0)
                dist_top = average(dist_top_values)
            except:
                print("NO TOF TOP")
            try:
                dist_front_values.append(int(tof_front.read()))
                if len(dist_front_values) >= 5:
                    dist_front_values.pop(0)
                dist_front = average(dist_front_values)
            except:
                print("NO TOF FRONT")
            try:
                x_gees = float(round(imu.accel.x, 1))
                y_gees = float(round(imu.accel.y, 1))
                z_gees = float(round(imu.accel.z, 1))
            except:
                print("NO IMU")
            try:
                v_read = int(Volt_Pin.read())
                volts = round(v_read * 5 / (4069/3.3), 1)
                a_read = int(Amps_Pin.read() )
                amps = round(a_read / 10 / (4069/3.3), 2)
            except:
                print("NO BATTERY")
            passed = time
        await asyncio.sleep_ms(100)

async def motor_control():
    global dist_top, dist_front, pwm_fwd, pwm_rvs, rpm, prev_error, integral, target_rpm
    pwm_fwd.freq(pwm_freq)
    pwm_rvs.freq(pwm_freq)
    passed = ticks_ms()
    while True:
        time = ticks_ms()
        interval = 50
        if (ticks_diff(time, passed) > interval):
            duty = target_rpm * 2
            pwm_fwd.duty(duty)
            pwm_rvs.duty(0)
            passed = time
        await asyncio.sleep_ms(100)

async def display1():
    i2c1 = SoftI2C(scl=Pin(pin_SCL1), sda=Pin(pin_SDA1), freq=100000)
    try:    
        oled1 = sh1106.SH1106_I2C(128, 64, i2c1, Pin(0), 0x3c)                              #rechts
        oled1.flip()
        oled1.fill(0)
        oled1.show()
    except:
        print("could not innit display 1")
    passed = ticks_ms()
    while True:
        global dist_top, dist_front, x_gees
        time = ticks_ms()
        interval = 250
        if  durchfahrt_aktiv == False:
            if (ticks_diff(time, passed) > interval) and durchfahrt_aktiv == False:            
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
        await asyncio.sleep_ms(200)
            
async def display2():
    i2c2 = SoftI2C(scl=Pin(pin_SCL2), sda=Pin(pin_SDA2), freq=100000)
    try:    
        oled2 = sh1106.SH1106_I2C(128, 64, i2c2, Pin(0), 0x3c)                              #links
        oled2.flip()
        oled2.fill(0)
        oled2.show()
    except:
        print("could not innit display 2")
    passed = ticks_ms()
    while True:
        global volts, amps, rpm
        time = ticks_ms()
        interval = 250
        if  durchfahrt_aktiv == False:
            if (ticks_diff(time, passed) > interval):       
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
        await asyncio.sleep_ms(200)

# ---------- STARTUP ----------

connect_wifi()
gc.enable()

rpm_pin_14.irq(trigger=Pin.IRQ_FALLING, handler=interrupt_handler)
print("warten auf mqtt input an adler/status/#")
client = MQTTClient(CLIENT_ID, MQTT_SERVER)
client.connect()
client.set_callback(mqtt_callback)
client.subscribe(MQTT_RECEIVE_TOPIC)
client.wait_msg()

print("starte in 5 sekunden")
sleep_ms(5000)

# ---------- LOOP ----------

try:
    loop.create_task(sensors())
    loop.create_task(motor_control())
    loop.create_task(display1())
    loop.create_task(display2())
    loop.create_task(abfahrt())
    loop.create_task(monitoring_send())
    loop.run_forever()

except Exception as e:
    print("Error:", e)

finally:
    pwm_fwd.duty(0)
    pwm_rvs.duty(0)
    loop.close()




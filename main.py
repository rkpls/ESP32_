
import time
import machine
import network

# ---------- VAR ----------

Board_Status = 0

pwm_freq = 200

rev_counter = 0
revs_sec = 0
min_rpm_sample_time = 2000
timer = 0
delay = 0

power = 0

# ---------- PIN ----------

Boot_Switch = machine.Pin(0, machine.Pin.IN)
Board_LED = machine.Pin(2, machine.Pin.OUT)

Pin19 = machine.Pin(19, machine.Pin.IN, machine.Pin.PULL_UP)

p22 = machine.Pin(22)
pwm22 = machine.PWM(p22)
p23 = machine.Pin(23)
pwm23 = machine.PWM(p23)

# ---------- DEF ----------

def interrupt_manual(Boot_Switch):
    global Board_Status
    Board_Status += 1
    print(f"Board Status: {Board_Status}")
def connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect('Zenbook-14-Pals','Micropython')
    else:
        pass
    print("Netzwerk: ", wlan.ifconfig)
def set_motor_speed():
    global power
    global duty
    duty = power / 100 * 1023                               #power reqest to in Hz
    if duty > 255:                                          #vorwärts min 25%
        pwm23.freq(pwm_freq)
        duty = int(duty)                                    #make integer
        pwm23.duty(duty)
        time.sleep(0.2)
    elif duty < -255:                                       #rückwärts min 25%
        pwm22.freq(pwm_freq)
        duty = int(duty*-1)
        pwm22.duty(duty)
        time.sleep(0.2)
    else:                                                   #motor aus
        pwm23.duty(0)
        pwm22.duty(0)
def interrupt_rev_counter(Pin19):
    global rev_counter
    rev_counter += 1
def LED_blink():
    Board_LED(1)
    time.sleep(0.2)
    Board_LED(0)
    time.sleep(0.2)    

# ---------- INTERRUPT ----------

Boot_Switch.irq(trigger=machine.Pin.IRQ_FALLING, handler=interrupt_manual)
Pin19.irq(trigger=machine.Pin.IRQ_RISING, handler=interrupt_rev_counter)

# ---------- WIFI ----------

def send_data():
    pass

# ---------- LOOP ----------

pwm23.duty(0)
pwm22.duty(0)

while Board_Status == 0:
    print("startup")
    connect()
    LED_blink()
    Board_Status += 1
        
# ---------- RUN ----------

while Board_Status == 1:
    print(f"Borad Status: {Board_Status}: Run")
    #power = int(input("Request Manual Power input "))
    while True:
        power = int(input())
        set_motor_speed()
        #send_data()
        
        time.sleep(0.2)                       #program delay
while Board_Status >= 2:
    power = 0
    set_motor_speed()
    print("idle")
    time.sleep(5)

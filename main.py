import time
import machine
import network
import usocket as socket

# ---------- VAR ----------

Board_Status = 0

pwm_freq = 200

last_request_time = 0
delay_btwn_request = 0.2

data = 0
power = 0
duty = 0

html_content = ""
terminal_log = 0

# ---------- PIN ----------

Board_LED = machine.Pin(38, machine.Pin.OUT)

p16 = machine.Pin(16)
pwm16 = machine.PWM(p16)
p17 = machine.Pin(17)
pwm17 = machine.PWM(p17)

# ---------- DEF ----------

# ---------- WIFI+DATA ----------

def handle_post_request(request):
    global data
    data_start = request.find(b'\r\n\r\n') + 4
    data = request[data_start:].decode()
    current_time = time.time()
    if current_time - last_request_time < delay_btwn_request:
        return


def server_loop():
    global serv
    conn, addr = serv.accept()
    request = conn.recv(1024)
    if b'GET / ' in request:
        response = 'HTTP/1.1 200 OK\nContent-Type: text/html\n\n' + html()
    elif b'POST /submit' in request:
        handle_post_request(request)
        response = 'HTTP/1.1 200 OK\nContent-Type: text/html\n\n' + html()
    else:
        response = 'HTTP/1.1 404 Not Found\nContent-Type: text/plain\n\nNot Found'
    conn.send(response)
    conn.close()

# ---------- MOTOR ----------

def set_motor_speed():
    global power, duty
    duty = power / 100 * 1023  # power request to in Hz
    if duty > 255:  # vorwärts min 25%
        pwm17.duty(0)
        pwm16.freq(pwm_freq)
        duty = int(duty)  # make integer
        pwm16.duty(duty)
    elif duty < -255:  # rückwärts min 25%
        pwm16.duty(0)
        pwm17.freq(pwm_freq)
        duty = int(duty * -1)
        pwm17.duty(duty)
    else:  # motor aus
        pwm16.duty(0)
        pwm17.duty(0)

# ---------- INTERRUPT ----------

# ---------- RESET ----------

# ----------- SETUP -----------

print("MAIN")
power = 0
pwm16.duty(0)
pwm17.duty(0)

# ---------- LOOP ----------

while True:
    server_loop()
    power = int(data)
    print(power)
    set_motor_speed()




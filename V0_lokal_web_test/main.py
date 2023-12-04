# ---------- LIBS ----------
import asyncio
import sys
import gc
import network
import ure as re
import time
import socket
import machine

with open("index.html", "r") as html_file:
    html_content = html_file.read()

# ---------- VAR ----------
wlan = network.WLAN(network.STA_IF)
ip = wlan.ifconfig()[0]

loop = asyncio.get_event_loop()

standard_delay = 200
last_request_time = time.time()

log_data = []
latest_log_data = str()

power = 0
duty = 0
pwm_freq = 1000

# ----------PIN----------
p16 = machine.Pin(16)
pwm16 = machine.PWM(p16)
p17 = machine.Pin(17)
pwm17 = machine.PWM(p17)

# ---------- FUNCTIONS ----------
def collect_terminal_log(log_list):
    global latest_log_data, log_data, last_request_time, standard_delay
    current_time = time.time()
    if current_time - last_request_time < standard_delay:
        return
    latest_log_data = f""
    log_data.append(latest_log_data)
    last_request_time = time.time()

# ---------- DATA_TRANSFER ----------
def handle_request(client_socket):
    global power
    try:
        request = client_socket.recv(1024).decode('utf-8')
        match = re.search(r'GET /update\?value=(\S+)', request)
        if match:
            value = int(match.group(1))
            power = value
        response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nOK"
        client_socket.send(response)
    except Exception as e:
        print("Error handling request:", e)
    finally:
        client_socket.close()


# ---------- MOTOR ----------s
def set_motor():
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



# ---------- SOCKET_THREAD ---------
async def task_socket():
    try:
        addr = socket.getaddrinfo(ip, 80)[0][-1]
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(addr)
        server_socket.listen(1)
        print("Server on {}:{}".format(*addr))

        while True:
            try:
                client_socket, addr = server_socket.accept()
                await loop.run_in_executor(None, handle_request, client_socket)
            except:
                print("No client handshake or error handling request")
                break
            await asyncio.sleep_ms(200)
    except Exception as e:
        print("Error in task_socket:", e)
    finally:
        server_socket.close()          



# ---------- MAIN----------

async def task_main():
    print("started Main")
    while True:
        
        set_motor()
        await asyncio.sleep_ms(20)



# ----------MONITOR----------
async def task_monitor():
    print("started Monitoring")
    while True:
        global power
        print(f"Free RAM:", gc.mem_free())
        print(power)
        await asyncio.sleep(2)

# ----------MAIN----------

print("MAIN")
if wlan.isconnected():
    print(ip)
    gc.enable()

try:
    loop.create_task(task_socket())
    loop.create_task(task_main())
    loop.create_task(task_monitor())
    loop.run_forever()
except Exception as e:
    print("Error:", e)
finally:
    pwm16.duty(0)
    pwm17.duty(0)
    loop.close()


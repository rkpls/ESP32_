# ---------- LIBS ----------
import uasyncio as asyncio
import os
import gc
import network
import time
import socket
import machine

with open("index.html", "r") as html_file:
    html_content = html_file.read()

# ---------- VAR ----------
wlan = network.WLAN(network.STA_IF)
ip = wlan.ifconfig()[0]

loop = asyncio.get_event_loop()

standard_delay = 0.5

log_data = []
latest_log_data = str()
last_request_time = time.time()

power = 0
duty = 0
pwm_freq = 500

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
    # No uhttpd references here

# ---------- DATA_TRANSFER ----------
def http_handler(request_str, client_sock):
    if request_str.startswith("GET /"):
        client_sock.send('HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n'.encode("utf-8"))
        client_sock.send(html_content.encode("utf-8"))
    elif request_str.startswith("POST /submit"):
        # Handle the POST request, e.g., updateLog on the server
        value = request_str.split("\r\n")[-1]  # Extract the value from the POST request
        print(value)
        client_sock.send('HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\n'.encode("utf-8"))
        client_sock.send("OK".encode("utf-8"))

# ---------- SOCKET_THREAD ----------
async def task_socket():
    try:
        addr = socket.getaddrinfo(ip, 80)[0][-1]
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(addr)
        s.listen(1)
        print("Server on {}:{}".format(*addr))
        while True:
            client_sock, _ = s.accept()
            # Read the HTTP request
            request_data = client_sock.recv(1024)
            request_str = request_data.decode("utf-8")
            # Check if it's a GET request
            if request_str.startswith("GET"):
                # Send the HTTP response
                http_handler(request_str, client_sock)
            client_sock.close()
            gc.collect()
            await asyncio.sleep_ms(500)

    except Exception as e:
        print("Error:", e)
        sys.exit()

# ---------- MOTOR ----------
async def task_motor():
    while True:
        global power, duty
        print(power)
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
        time.sleep(5)
        await asyncio.sleep_ms(200)

# ---------- MAIN_THREAD ----------
print("MAIN")
if wlan.isconnected():
    print(ip)

power = 0
pwm16.duty(0)
pwm17.duty(0)

try:
    loop.create_task(task_socket())
    loop.create_task(task_motor())
    loop.run_forever()
except Exception as e:
    print("Error:", e)
finally:
    pwm16.duty(0)
    pwm17.duty(0)
    loop.close()





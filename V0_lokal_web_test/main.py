# ---------- LIBS ----------
import asyncio
import network
import time
import websocket
import socket
import sys
import machine

with open("index.html", "r") as html_file:
    html_content = html_file.read()

# ---------- VAR ----------
ssid = 'Zenfone_9'
password = 'Micropython'

standard_delay = 0.5

log_data = []
latest_log_data = str()
last_request_time = time.time()  # Initialize last_request_time

# ---------- FUNCTIONS ----------

def collect_terminal_log(log_list):
    global latest_log_data, log_data, last_request_time, standard_delay
    current_time = time.time()
    if current_time - last_request_time < standard_delay:
        return
    latest_log_data = f""
    log_data.append(latest_log_data)
    last_request_time = time.time()
    send_data_web(latest_log_data)

def data_bat_voltage():
    pass

def data_bat_current():
    pass

def data_rpm():
    pass

# ---------- DATA_TRANSFER ----------

def read_data_web():
    # Code to read data from the sensor
    pass

def send_data_web():
    # Code to send data to the web server
    pass

# ---------- SOCKET_THREAD ----------
def task_socket():
    time.sleep(0.1)
    try:
        addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(addr)
        s.listen(1)
        print("Server started on {}:{}".format(*addr))
        while True:
            client_sock, client_addr = s.accept()
            print("Request from:", client_addr)
            # Read the HTTP request
            request_data = client_sock.recv(1024)
            request_str = request_data.decode("utf-8")
            # Check if it's a GET request
            if request_str.startswith("GET"):
                # Send the HTTP response
                response = 'HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n' + html_content
                client_sock.send(response.encode("utf-8"))
            client_sock.close()
    except Exception as e:
        print("Error:", e)
        sys.exit()

# ---------- MAIN_THREAD ----------

while True:
    task_socket()

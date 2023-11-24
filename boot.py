
import network
import socket
                   
def connect():
    wifi = network.WLAN(network.STA_IF)                 #STA = Station Mode
    wifi.active(True)
    if wifi.isconnected(False):
        wifi.connect('Zenbook-14-Pals', 'Micropython')  # Network intel
    else:
        pass
    wifi.ifconfig()  # output IP

def start_socket():
    pass

print("BOOT")
print("Connecting to Wifi")
connect()
print("Starting Webserver Socket")
start_socket()


import main         # run main.py

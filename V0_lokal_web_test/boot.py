#---------- LIBS ----------
import network
import socket
import os

#---------- WIFI ----------
def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())

def get_ip_address():
    wlan = network.WLAN(network.STA_IF)
    if wlan.isconnected():
        return wlan.ifconfig()[0]
    else:
        return None

#---------- PAGE ----------
def start_webpage(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((ip, port))
    s.listen(5)
    print('started webpage on port', port)

    while True:
        client, addr = s.accept()
        print('got a connection from %s:%d' % (addr[0], addr[1]))
        client.send('HTTP/1.1 200 OK\r\n')
        client.send('Content-Type: text/html\r\n')
        client.send('Connection: close\r\n\r\n')
        client.sendall('<html><body><pre>')
        for line in os.popen('log'):
            client.sendall(line)
        client.sendall('</pre></body></html>\r\n')
        client.close()


#---------- RUN ----------
print("BOOT")
connect_wifi('Zenfone_9', 'Micropython')
import main
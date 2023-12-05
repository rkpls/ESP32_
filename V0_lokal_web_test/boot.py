import network

ssid = 'Zenbook-14-Pals'
password = 'Micropython'

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass

connect_wifi()
import main
#---------- LIBS ----------
import network

#---------- WIFI ----------
def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())

#---------- RUN ----------
print("BOOT")
connect_wifi('Zenfone_9', 'Micropython')
import main
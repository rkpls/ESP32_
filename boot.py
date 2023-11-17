import network
def connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect('Zenbook-14-Pals','Micropython')
    else:
        pass
    print("Netzwerk: ", wlan.ifconfig)
    
connect()
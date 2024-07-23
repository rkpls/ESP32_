import gc
from machine import Pin, PWM, SoftI2C, I2S, unique_id, reset
from utime import ticks_ms, ticks_diff, sleep_ms, localtime
import asyncio

import network
from ubinascii import hexlify
from umqtt.simple import MQTTClient
import json


ssid = '***'                                           #Schulwlan
password = '***'
wlan = network.WLAN(network.STA_IF)

MQTT_SERVER = '192.168.2.45'
CLIENT_ID = hexlify(unique_id())
MQTT_RECEIVE_TOPIC = 'status/#'

loop = asyncio.get_event_loop()

status = False

def connect_wifi():                                         #fn für WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    print(wlan.ifconfig())
    return wlan

def mqtt_callback(topic, msg):
    global status
    print("Received MQTT message:", msg.decode())
    status = True
    print(status)

async def reset_status():
    global status
    while True:
        sleep_ms(5000)
        if status == True:
            status = False
            print("Status reset to False")
        await asyncio.sleep_ms(10)

async def waitformessage():
    while True:
        client.wait_msg()
        await asyncio.sleep_ms(10)

connect_wifi()

client = MQTTClient(CLIENT_ID, MQTT_SERVER)
client.connect()
client.set_callback(mqtt_callback)
client.subscribe(MQTT_RECEIVE_TOPIC)

try:
    loop.create_task(reset_status())
    loop.create_task(waitformessage())
    loop.run_forever()
except Exception as e:
    print("Error:", e)
finally:
    sleep_ms(5000)
    loop.close()

        

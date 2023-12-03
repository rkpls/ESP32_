# ---------- LIBS ----------

import asyncio
import time
import websocket
import ujson
import machine

# ----------FUNCTIONS----------

class TerminalLogger:
    def __init__(self):
        self.log = []

    def write(self, message):
        self.log.append((time.time(), message))

# ----------SOCKET_THREAD----------
async def task_socket(request, response):
    websocket = await request.accept()
    print("WebSocket connection established")

    try:
        while True:
            data = await websocket.recv()
            if data is None:
                break
            # Simulate processing and send a response back
            response_data = {"status": "OK", "message": f"Received: {data}"}
            await websocket.send(ujson.dumps(response_data))
    except websocket.WebSocketClosed:
        print("WebSocket connection closed")


# ----------WEB_THREAD----------
async def task_web(request, response):
    if request.url == "/hyperloop":
        # Simulate sending terminal log data as JSON
        log_data = ["Log message 1", "Log message 2", "Log message 3"]
        await response.send(ujson.dumps(log_data))
    elif request.url == "/ws":
        # Handle WebSocket connection
        await task_socket(request, response)
    else:
        await response.send("404 Not Found")


# ----------MAIN_THREAD----------
async def task_main():
    while True:
        print("Task Main is running")
        await asyncio.sleep(5)


# ----------LOOP_MANAGE-----------
loop = asyncio.get_event_loop()

loop.create_task(task_socket)
loop.create_task(task_web)
loop.create_task(task_main)

loop.run_forever()
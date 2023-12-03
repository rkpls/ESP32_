import socket
import uasyncio as asyncio

async def run_server(ip, port):
    loop = asyncio.get_event_loop()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((ip, port))
    server.listen(1)

    while True:
        client, addr = await loop.sock_accept(server)
        print("Client connected")
        loop.create_task(handle_client(client))

async def handle_client(client):
    loop = asyncio.get_event_loop()

    while True:
        data = await loop.sock_recv(client, 1024)
        if not data:
            break
        process_data(data)

    print("Client disconnected")
    client.close()

def process_data(data):
    # Handle the received data, e.g., execute a command, read a sensor, etc.
    pass

loop = asyncio.get_event_loop()
loop.run_until_complete(run_server('80.187.124.97', 81))
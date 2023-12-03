# ---------- LIBS ----------

import asyncio
import time
import websocket
import machine

# ----------FUNCTIONS----------

class TerminalLogger:
    def __init__(self):
        self.log = []

    def write(self, message):
        self.log.append((time.time(), message))

# ----------WEB_THREAD----------
async def task_web():
    while True:
        global
        print(log)
        terminal_logger = TerminalLogger()
        sys.stdout = terminal_logger
        current_time = time.time()
        last_second_log = [(timestamp, message) for timestamp, message in terminal_logger.log if current_time - timestamp <= 1]

# ----------MAIN_THREAD----------
async def task_main():
    while True:
        print("Task Main is running")
        await asyncio.sleep(5)

# ----------LOOP_MANAGE-----------
loop = asyncio.get_event_loop()

loop.create_task(task_web())
loop.create_task(task_main())

loop.run_forever()
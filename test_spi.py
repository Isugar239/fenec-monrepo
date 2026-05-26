# Raspberry Pi as SPI Master

import spidev
import time

# Open SPI bus 0, device (CS) 0
spi = spidev.SpiDev()
spi.open(0, 0)

# Set SPI speed and mode
spi.max_speed_hz = 50000
spi.mode = 0

def send_data(data):
    """Send a single byte to the SPI slave"""
    response = spi.xfer2([data])
    return response

try:
    while True:
        data = 42  # Example data byte
        print(f"Sending: {data}")
        resp = send_data(data)
        print(f"Received: {resp[0]}")
        time.sleep(1)

except KeyboardInterrupt:
    spi.close()

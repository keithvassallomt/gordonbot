import spidev
import time

# Open SPI bus 0, device 0 (which is CE0)
spi = spidev.SpiDev()
spi.open(0, 0)  # (bus 0, device 0 = CE0)
spi.max_speed_hz = 1350000

def read_channel(channel):
    # MCP3008 protocol: start bit, single/diff bit, channel (3 bits), 5 'don't care' bits
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

# Example: Read channel 0
while True:
    value = read_channel(0)
    voltage = (value * 3.3) / 1023
    print('Raw ADC Value:', value)
    print('ADC Voltage: {:.2f} V'.format(voltage))
    time.sleep(1)
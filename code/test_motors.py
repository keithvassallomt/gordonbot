from gpiozero import OutputDevice
from time import sleep

# Motor A
AIN1 = OutputDevice(5)
AIN2 = OutputDevice(6)

# Motor B
BIN1 = OutputDevice(13)
BIN2 = OutputDevice(19)

def forward():
    print("Motors forward")
    AIN1.on()
    AIN2.off()
    BIN1.on()
    BIN2.off()

def backward():
    print("Motors backward")
    AIN1.off()
    AIN2.on()
    BIN1.off()
    BIN2.on()

def stop():
    print("Motors stop")
    AIN1.off()
    AIN2.off()
    BIN1.off()
    BIN2.off()

try:
    forward()
    sleep(2)
    stop()
    sleep(1)
    backward()
    sleep(2)
    stop()
except KeyboardInterrupt:
    stop()
from gpiozero import OutputDevice
from time import sleep

MA_M1 = OutputDevice(5)   # DRV8833 MA_M1
MA_M2 = OutputDevice(6)   # DRV8833 MA_M2
MB_M1 = OutputDevice(13)  # DRV8833 MB_M1
MB_M2 = OutputDevice(19)  # DRV8833 MB_M2

def coast():
    MA_M1.off(); MA_M2.off(); MB_M1.off(); MB_M2.off()

print("Motor A forward 2s")
MA_M1.on(); MA_M2.off(); sleep(2); coast(); sleep(1)

print("Motor A reverse 2s")
MA_M1.off(); MA_M2.on(); sleep(2); coast(); sleep(1)

print("Motor B forward 2s")
MB_M1.on(); MB_M2.off(); sleep(2); coast(); sleep(1)

print("Motor B reverse 2s")
MB_M1.off(); MB_M2.on(); sleep(2); coast()
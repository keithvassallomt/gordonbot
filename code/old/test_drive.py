from gpiozero import OutputDevice
from time import sleep

MA_M1 = OutputDevice(5)   # DRV8833 MA_M1
MA_M2 = OutputDevice(6)   # DRV8833 MA_M2
MB_M1 = OutputDevice(13)  # DRV8833 MB_M1
MB_M2 = OutputDevice(19)  # DRV8833 MB_M2

def coast():
    MA_M1.off(); MA_M2.off(); MB_M1.off(); MB_M2.off()

print("Forward 2s")
MA_M1.off();
MB_M1.off();

MA_M2.on(); 
MB_M2.on();
sleep(1); 
coast();
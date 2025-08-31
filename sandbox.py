from gpiozero import Motor
from time import sleep

# DRV8833 inputs (BCM numbering)
# Motor A on AIN1/AIN2 -> GPIO 5/6
# Motor B on BIN1/BIN2 -> GPIO 13/19
motorA = Motor(forward=5, backward=6)
motorB = Motor(forward=19, backward=13)

try:
    print("Forward 2s")
    motorA.forward(0.5)
    motorB.forward(0.5)
    sleep(2)

    # brief stop between directions (optional but nice for drivers)
    motorA.stop(); motorB.stop()
    sleep(0.3)

    print("Backward 2s")
    motorA.backward(0.5)
    motorB.backward(0.5)
    sleep(2)

finally:
    motorA.stop()
    motorB.stop()
    print("Done.")
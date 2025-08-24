from gpiozero import Button, OutputDevice
from time import sleep, time
import math

# Motor A control pins
AIN1 = OutputDevice(5)
AIN2 = OutputDevice(6)

# Encoder A phase pin
encoder_A = Button(22)

# Constants
WHEEL_DIAMETER_CM = 3.0  # adjust to your actual wheel
PULSES_PER_REVOLUTION = 700  # Updated to reflect gear reduction and pulse rate

pulse_count = 0

def count_pulse():
    global pulse_count
    pulse_count += 1

encoder_A.when_pressed = count_pulse

print("Running Motor A forward for 2 seconds while counting pulses...")

# Spin motor forward
AIN1.on()
AIN2.off()

start_time = time()
sleep(2)
end_time = time()

# Stop motor
AIN1.off()
AIN2.off()

# Calculate distance
revolutions = pulse_count / PULSES_PER_REVOLUTION
circumference = math.pi * WHEEL_DIAMETER_CM
distance = revolutions * circumference

print(f"Pulses: {pulse_count}")
print(f"Revolutions: {revolutions:.2f}")
print(f"Distance travelled: {distance:.2f} cm")
import time, lgpio

ENCODER_RIGHT_PA = 12
ENCODER_RIGHT_PB = 27

handle = lgpio.gpiochip_open(0)
pin_a, pin_b = ENCODER_RIGHT_PA, ENCODER_RIGHT_PB
for _ in range(400):
    print(f"{lgpio.gpio_read(handle, pin_a)} {lgpio.gpio_read(handle, pin_b)}")
    time.sleep(0.05)
lgpio.gpiochip_close(handle)

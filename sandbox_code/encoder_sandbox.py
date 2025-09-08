import time
import math
import logging
import argparse
import gc

try:
    import RPi.GPIO as GPIO
except Exception as e:
    raise SystemExit(
        "RPi.GPIO is required to run this script on a Raspberry Pi.\n"
        f"Import error: {e}"
    )


# If another module defines MOTOR_RIGHT_B, you can import it instead.
# from some_module import MOTOR_RIGHT_B

# Logging: write to file and console; truncate file each run
logger = logging.getLogger("sandbox2")
logger.setLevel(logging.INFO)
_file_handler = logging.FileHandler("sandbox2_out.log", mode="w")  # clear on each run
_console_handler = logging.StreamHandler()
_fmt = logging.Formatter("%(message)s")
_file_handler.setFormatter(_fmt)
_console_handler.setFormatter(_fmt)
logger.handlers.clear()
logger.addHandler(_file_handler)
logger.addHandler(_console_handler)

# Default mappings on BCM pins (IN1, IN2)
MOTOR_LEFT_A = (19, 13)
ENCODER_LEFT_A = (12, 27)  # (PHASE_A, PHASE_B)

MOTOR_RIGHT_B = (5, 6)
ENCODER_RIGHT_B = (22, 26)  # (PHASE_A, PHASE_B)


# --- Distance calibration (adjust to your hardware) ---
# We count every valid quadrature state transition (x4 decoding).
# Provide motor-shaft CPR and gear ratio, and we compute steps/rev.
# Use the drive sprocket's pitch diameter (track drive) for distance.
DRIVE_SPROCKET_PITCH_DIAMETER_MM = 30.0  # drive sprocket pitch diameter in mm
# Manufacturer states 12 counts/rev at motor shaft WHEN counting both edges of both channels (x4).
# So 12 already includes x4 quadrature. Output shaft counts/rev = 12 * gear_ratio at x4.
ENCODER_CPR_MOTOR_SHAFT_X4 = 12.0
GEAR_RATIO = 100.3710  # gearbox reduction
# Decoding factor: 1 for x1 (A rising only), 2 for x2, 4 for x4.
# Default to x1 for reliable Python callback performance at higher speeds.
DECODING_FACTOR = 1
# Software deglitching/debounce (microseconds). Filters spurious rapid edges.
DEBOUNCE_US = 1200
# Empirical scale factors to correct for track slip / effective pitch diameter.
# These apply per direction since many drivetrains have asymmetry in reverse vs forward.
CALIBRATION_SCALE_FWD = 1.024
CALIBRATION_SCALE_REV = 1.068

# Derived: steps per output shaft revolution for chosen decoding
ENCODER_STEPS_PER_REV = int(round(GEAR_RATIO * ENCODER_CPR_MOTOR_SHAFT_X4 * (DECODING_FACTOR / 4.0)))



class TestMotor:
    def __init__(self, motor_pins, encoder_pins=None, name=None):
        """
        motor_pins: tuple[int, int] -> (IN1, IN2) on BCM numbering
        encoder_pins: optional tuple[int, int] -> (PHASE_A, PHASE_B)
        name: optional name for debug prints
        """
        self.in1, self.in2 = motor_pins
        self.name = name or "MOTOR"
        self.enc_a = None
        self.enc_b = None

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)

        # PWM for speed control
        self._pwm_freq_hz = 1000
        self._pwm1 = GPIO.PWM(self.in1, self._pwm_freq_hz)
        self._pwm2 = GPIO.PWM(self.in2, self._pwm_freq_hz)
        self._pwm1.start(0)
        self._pwm2.start(0)

        if encoder_pins is not None:
            self.enc_a, self.enc_b = encoder_pins
            # Configure encoder inputs with pull-ups to avoid floating
            GPIO.setup(self.enc_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.enc_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def spin_forward(self, seconds: float = 2.0, speed_percent: float = 50.0):
        """Spin forward: IN1=HIGH, IN2=LOW for given seconds."""
        return self._spin_for(seconds, forward=True, speed_percent=speed_percent)

    def spin_backward(self, seconds: float = 2.0, speed_percent: float = 50.0):
        """Spin backward: IN1=LOW, IN2=HIGH for given seconds."""
        return self._spin_for(seconds, forward=False, speed_percent=speed_percent)

    def spin(self, seconds: float = 2.0, speed_percent: float = 50.0):
        """Alias for forward spin to retain compatibility."""
        return self.spin_forward(seconds, speed_percent=speed_percent)

    def stop(self):
        # Set both PWM duties to 0 and pull outputs low
        try:
            self._pwm1.ChangeDutyCycle(0)
            self._pwm2.ChangeDutyCycle(0)
        except Exception:
            pass
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def shutdown(self):
        # Stop PWM channels before GPIO.cleanup()
        try:
            self._pwm1.stop()
        except Exception:
            pass
        try:
            self._pwm2.stop()
        except Exception:
            pass
        # Drop references so __del__ runs now (while modules are intact)
        try:
            p1 = self._pwm1
            p2 = self._pwm2
            self._pwm1 = None
            self._pwm2 = None
            del p1
            del p2
        except Exception:
            pass

    # --- Encoder support (interrupt-driven for accuracy) ---
    def _spin_for(self, seconds: float, forward: bool, speed_percent: float) -> int:
        duty = max(0.0, min(100.0, float(speed_percent)))
        if forward:
            # Drive in1 with PWM, hold in2 low
            GPIO.output(self.in2, GPIO.LOW)
            self._pwm1.ChangeDutyCycle(duty)
            self._pwm2.ChangeDutyCycle(0)
        else:
            # Drive in2 with PWM, hold in1 low
            GPIO.output(self.in1, GPIO.LOW)
            self._pwm2.ChangeDutyCycle(duty)
            self._pwm1.ChangeDutyCycle(0)

        deadline = time.time() + seconds
        steps_abs = 0  # count of valid encoder steps (per decoding mode)

        # x1: count rising edges on A only. x4: both edges on A and B using state machine.
        next_map = {0: 1, 1: 3, 3: 2, 2: 0}
        prev_map = {0: 2, 2: 3, 3: 1, 1: 0}
        prev_state = None

        if self.enc_a is not None and self.enc_b is not None:
            a0 = GPIO.input(self.enc_a)
            b0 = GPIO.input(self.enc_b)
            prev_state = (a0 << 1) | b0

        debounce_ns = int(DEBOUNCE_US * 1000)
        last_change_ns = 0

        def on_edge_x4(channel):
            nonlocal steps_abs, prev_state
            if self.enc_a is None or self.enc_b is None:
                return
            now_ns = time.monotonic_ns()
            if debounce_ns > 0 and (now_ns - last_change_ns) < debounce_ns:
                return
            a = GPIO.input(self.enc_a)
            b = GPIO.input(self.enc_b)
            state = (a << 1) | b
            if state != prev_state:
                if next_map.get(prev_state) == state or prev_map.get(prev_state) == state:
                    steps_abs += 1
                prev_state = state
                last_change_ns = now_ns

        def on_edge_x1(channel):
            nonlocal steps_abs
            # Only count rising edges on channel A
            if channel == self.enc_a:
                now_ns = time.monotonic_ns()
                if debounce_ns > 0 and (now_ns - getattr(self, "_last_a_ns", 0)) < debounce_ns:
                    return
                steps_abs += 1
                self._last_a_ns = now_ns

        try:
            # Attach callbacks per decoding mode
            if DECODING_FACTOR >= 4:
                if self.enc_a is not None:
                    GPIO.add_event_detect(self.enc_a, GPIO.BOTH, callback=on_edge_x4)
                if self.enc_b is not None:
                    GPIO.add_event_detect(self.enc_b, GPIO.BOTH, callback=on_edge_x4)
            else:
                if self.enc_a is not None:
                    GPIO.add_event_detect(self.enc_a, GPIO.RISING, callback=on_edge_x1)

            # Run for the allotted time
            while time.time() < deadline:
                time.sleep(0.002)
        finally:
            # Remove callbacks and stop
            try:
                if self.enc_a is not None:
                    GPIO.remove_event_detect(self.enc_a)
            except Exception:
                pass
            try:
                if self.enc_b is not None and DECODING_FACTOR >= 4:
                    GPIO.remove_event_detect(self.enc_b)
            except Exception:
                pass
            self.stop()
        return steps_abs


def spin_both(left_motor: "TestMotor", right_motor: "TestMotor", seconds: float, forward: bool, speed_percent: float):
    """Spin two motors simultaneously for a duration, counting steps for each.

    Returns (steps_left_abs, steps_right_abs).
    """
    # Drive direction
    duty = max(0.0, min(100.0, float(speed_percent)))
    if forward:
        # FWD: PWM on in1 for both, in2 low
        GPIO.output(left_motor.in2, GPIO.LOW)
        GPIO.output(right_motor.in2, GPIO.LOW)
        left_motor._pwm1.ChangeDutyCycle(duty)
        right_motor._pwm1.ChangeDutyCycle(duty)
        left_motor._pwm2.ChangeDutyCycle(0)
        right_motor._pwm2.ChangeDutyCycle(0)
    else:
        # REV: PWM on in2 for both, in1 low
        GPIO.output(left_motor.in1, GPIO.LOW)
        GPIO.output(right_motor.in1, GPIO.LOW)
        left_motor._pwm2.ChangeDutyCycle(duty)
        right_motor._pwm2.ChangeDutyCycle(duty)
        left_motor._pwm1.ChangeDutyCycle(0)
        right_motor._pwm1.ChangeDutyCycle(0)

    deadline = time.time() + seconds

    # Encoder step counts
    steps_l = 0
    steps_r = 0

    # x4 helpers (only used if DECODING_FACTOR >= 4)
    next_map = {0: 1, 1: 3, 3: 2, 2: 0}
    prev_map = {0: 2, 2: 3, 3: 1, 1: 0}
    prev_state_l = None
    prev_state_r = None

    if left_motor.enc_a is not None and left_motor.enc_b is not None:
        la0 = GPIO.input(left_motor.enc_a)
        lb0 = GPIO.input(left_motor.enc_b)
        prev_state_l = (la0 << 1) | lb0
    if right_motor.enc_a is not None and right_motor.enc_b is not None:
        ra0 = GPIO.input(right_motor.enc_a)
        rb0 = GPIO.input(right_motor.enc_b)
        prev_state_r = (ra0 << 1) | rb0

    debounce_ns = int(DEBOUNCE_US * 1000)
    last_change_ns_l = 0
    last_change_ns_r = 0

    def on_edge_left_x4(channel):
        nonlocal steps_l, prev_state_l, last_change_ns_l
        if left_motor.enc_a is None or left_motor.enc_b is None:
            return
        now_ns = time.monotonic_ns()
        if debounce_ns > 0 and (now_ns - last_change_ns_l) < debounce_ns:
            return
        la = GPIO.input(left_motor.enc_a)
        lb = GPIO.input(left_motor.enc_b)
        state_l = (la << 1) | lb
        if state_l != prev_state_l:
            if next_map.get(prev_state_l) == state_l or prev_map.get(prev_state_l) == state_l:
                steps_l += 1
            prev_state_l = state_l
            last_change_ns_l = now_ns

    def on_edge_right_x4(channel):
        nonlocal steps_r, prev_state_r, last_change_ns_r
        if right_motor.enc_a is None or right_motor.enc_b is None:
            return
        now_ns = time.monotonic_ns()
        if debounce_ns > 0 and (now_ns - last_change_ns_r) < debounce_ns:
            return
        ra = GPIO.input(right_motor.enc_a)
        rb = GPIO.input(right_motor.enc_b)
        state_r = (ra << 1) | rb
        if state_r != prev_state_r:
            if next_map.get(prev_state_r) == state_r or prev_map.get(prev_state_r) == state_r:
                steps_r += 1
            prev_state_r = state_r
            last_change_ns_r = now_ns

    def on_edge_left_x1(channel):
        nonlocal steps_l
        if channel == left_motor.enc_a:
            now_ns = time.monotonic_ns()
            if debounce_ns > 0 and (now_ns - getattr(left_motor, "_last_a_ns", 0)) < debounce_ns:
                return
            steps_l += 1
            left_motor._last_a_ns = now_ns

    def on_edge_right_x1(channel):
        nonlocal steps_r
        if channel == right_motor.enc_a:
            now_ns = time.monotonic_ns()
            if debounce_ns > 0 and (now_ns - getattr(right_motor, "_last_a_ns", 0)) < debounce_ns:
                return
            steps_r += 1
            right_motor._last_a_ns = now_ns

    try:
        if DECODING_FACTOR >= 4:
            if left_motor.enc_a is not None:
                GPIO.add_event_detect(left_motor.enc_a, GPIO.BOTH, callback=on_edge_left_x4)
            if left_motor.enc_b is not None:
                GPIO.add_event_detect(left_motor.enc_b, GPIO.BOTH, callback=on_edge_left_x4)
            if right_motor.enc_a is not None:
                GPIO.add_event_detect(right_motor.enc_a, GPIO.BOTH, callback=on_edge_right_x4)
            if right_motor.enc_b is not None:
                GPIO.add_event_detect(right_motor.enc_b, GPIO.BOTH, callback=on_edge_right_x4)
        else:
            if left_motor.enc_a is not None:
                GPIO.add_event_detect(left_motor.enc_a, GPIO.RISING, callback=on_edge_left_x1)
            if right_motor.enc_a is not None:
                GPIO.add_event_detect(right_motor.enc_a, GPIO.RISING, callback=on_edge_right_x1)

        while time.time() < deadline:
            time.sleep(0.002)
    finally:
        try:
            if left_motor.enc_a is not None:
                GPIO.remove_event_detect(left_motor.enc_a)
        except Exception:
            pass
        try:
            if left_motor.enc_b is not None and DECODING_FACTOR >= 4:
                GPIO.remove_event_detect(left_motor.enc_b)
        except Exception:
            pass
        try:
            if right_motor.enc_a is not None:
                GPIO.remove_event_detect(right_motor.enc_a)
        except Exception:
            pass
        try:
            if right_motor.enc_b is not None and DECODING_FACTOR >= 4:
                GPIO.remove_event_detect(right_motor.enc_b)
        except Exception:
            pass
        left_motor.stop()
        right_motor.stop()

    return steps_l, steps_r


def main():
    parser = argparse.ArgumentParser(description="Motor/encoder sandbox2")
    parser.add_argument(
        "--both",
        action="store_true",
        help="Drive both motors simultaneously forwards for 2s then backwards for 2s",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=50.0,
        help="Motor speed as duty cycle percent (0-100), default 50",
    )
    parser.add_argument(
        "--decoding",
        choices=["x1", "x4"],
        default=None,
        help="Encoder decode mode: x1 (A rising only) or x4 (both edges on A and B)",
    )
    parser.add_argument(
        "--debounce-us",
        type=int,
        default=None,
        help="Software debounce in microseconds (default 1200)",
    )
    parser.add_argument(
        "--scale-fwd",
        type=float,
        default=None,
        help="Empirical calibration scale for forward distance (multiplier)",
    )
    parser.add_argument(
        "--scale-rev",
        type=float,
        default=None,
        help="Empirical calibration scale for reverse distance (multiplier)",
    )
    args = parser.parse_args()

    speed = max(0.0, min(100.0, float(args.speed)))
    # Allow override of global decoding factor via CLI
    global DECODING_FACTOR
    if args.decoding == "x4":
        DECODING_FACTOR = 4
    elif args.decoding == "x1":
        DECODING_FACTOR = 1
    # Debounce override
    global DEBOUNCE_US
    if args.debounce_us is not None:
        DEBOUNCE_US = max(0, int(args.debounce_us))
    scale_fwd = CALIBRATION_SCALE_FWD if args.scale_fwd is None else float(args.scale_fwd)
    scale_rev = CALIBRATION_SCALE_REV if args.scale_rev is None else float(args.scale_rev)

    # Recompute steps/rev after potential decode override
    global ENCODER_STEPS_PER_REV
    ENCODER_STEPS_PER_REV = int(round(GEAR_RATIO * ENCODER_CPR_MOTOR_SHAFT_X4 * (DECODING_FACTOR / 4.0)))

    if args.both:
        left_motor = TestMotor(MOTOR_LEFT_A, encoder_pins=ENCODER_LEFT_A, name="LEFT_A")
        right_motor = TestMotor(MOTOR_RIGHT_B, encoder_pins=ENCODER_RIGHT_B, name="RIGHT_B")

        logger.info(f"SPINNING BOTH MOTORS FORWARDS FOR 2 SECONDS @ {speed:.0f}%")
        steps_left_fwd, steps_right_fwd = spin_both(left_motor, right_motor, 2.0, forward=True, speed_percent=speed)
        time.sleep(2.0)
        logger.info(f"SPINNING BOTH MOTORS BACKWARDS FOR 2 SECONDS @ {speed:.0f}%")
        steps_left_rev, steps_right_rev = spin_both(left_motor, right_motor, 2.0, forward=False, speed_percent=speed)
    else:
        left_motor = TestMotor(MOTOR_LEFT_A, encoder_pins=ENCODER_LEFT_A, name="LEFT_A")
        logger.info(f'SPINNING MOTOR LEFT A FORWARDS FOR 2 SECONDS @ {speed:.0f}%')
        steps_left_fwd = left_motor.spin_forward(2.0, speed_percent=speed)
        time.sleep(2.0)
        logger.info(f'SPINNING MOTOR LEFT A BACKWARDS FOR 2 SECONDS @ {speed:.0f}%')
        steps_left_rev = left_motor.spin_backward(2.0, speed_percent=speed)

        right_motor = TestMotor(MOTOR_RIGHT_B, encoder_pins=ENCODER_RIGHT_B, name="RIGHT_B")
        logger.info(f'SPINNING MOTOR RIGHT B FORWARDS FOR 2 SECONDS @ {speed:.0f}%')
        steps_right_fwd = right_motor.spin_forward(2.0, speed_percent=speed)
        time.sleep(2.0)
        logger.info(f'SPINNING MOTOR RIGHT B BACKWARDS FOR 2 SECONDS @ {speed:.0f}%')
        steps_right_rev = right_motor.spin_backward(2.0, speed_percent=speed)

    # Convert steps to millimeters
    if ENCODER_STEPS_PER_REV <= 0:
        raise SystemExit("ENCODER_STEPS_PER_REV must be > 0 for distance calculation")
    circumference_mm = math.pi * DRIVE_SPROCKET_PITCH_DIAMETER_MM
    mm_per_step_base = circumference_mm / ENCODER_STEPS_PER_REV

    # Report calibration context
    logger.info(
        f"CALIBRATION: drive_sprocket_pitch_diam={DRIVE_SPROCKET_PITCH_DIAMETER_MM:.1f}mm, gear={GEAR_RATIO}, motor_cpr_x4={ENCODER_CPR_MOTOR_SHAFT_X4}, decode_factor={DECODING_FACTOR}, debounce_us={DEBOUNCE_US}, steps/rev_out={ENCODER_STEPS_PER_REV}, base_mm/step={mm_per_step_base:.4f}, scale_fwd={scale_fwd:.3f}, scale_rev={scale_rev:.3f}, mm/step_fwd={mm_per_step_base*scale_fwd:.4f}, mm/step_rev={mm_per_step_base*scale_rev:.4f}"
    )
    logger.info(f"SPEED: commanded PWM duty = {speed:.0f}%")

    left_fwd_mm = steps_left_fwd * mm_per_step_base * scale_fwd
    left_rev_mm = steps_left_rev * mm_per_step_base * scale_rev
    right_fwd_mm = steps_right_fwd * mm_per_step_base * scale_fwd
    right_rev_mm = steps_right_rev * mm_per_step_base * scale_rev

    avg_fwd_mm = (left_fwd_mm + right_fwd_mm) / 2.0
    avg_rev_mm = (left_rev_mm + right_rev_mm) / 2.0

    logger.info(
        f"STEPS: LEFT_A FWD={steps_left_fwd} REV={steps_left_rev} | RIGHT_B FWD={steps_right_fwd} REV={steps_right_rev}"
    )
    logger.info(
        f"LEFT_A distance: FWD={left_fwd_mm:.1f} mm, REV={left_rev_mm:.1f} mm"
    )
    logger.info(
        f"RIGHT_B distance: FWD={right_fwd_mm:.1f} mm, REV={right_rev_mm:.1f} mm"
    )
    logger.info(
        f"AVERAGE distance (both motors): FWD={avg_fwd_mm:.1f} mm, REV={avg_rev_mm:.1f} mm"
    )

    # Stop PWM channels cleanly
    try:
        left_motor.shutdown()
    except Exception:
        pass
    try:
        right_motor.shutdown()
    except Exception:
        pass

    GPIO.cleanup()
    # Ensure any lingering PWM objects finalize before interpreter teardown
    try:
        del left_motor
        del right_motor
    except Exception:
        pass
    gc.collect()


if __name__ == "__main__":
    main()

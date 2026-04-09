import serial, time

PORT = 'COM4'
BAUD = 115200
HEAD = 0xFF
DEVICE_ID = 0xFC
COMPLEMENT = 257 - DEVICE_ID
NEUTRAL = 90  # Bidirectional ESC neutral / stop angle

def send_cmd(ser, cmd):
    checksum = sum(cmd, COMPLEMENT) & 0xFF
    cmd.append(checksum)
    ser.write(bytearray(cmd))

def set_pwm_servo(ser, servo_id, angle):
    angle = max(0, min(180, int(angle)))
    cmd = [HEAD, DEVICE_ID, 0x00, 0x03, int(servo_id), angle]
    cmd[2] = len(cmd) - 1
    send_cmd(ser, cmd)

def stop_motors(ser, duration=2.0):
    """Send neutral to both ESCs continuously for the given duration."""
    end = time.monotonic() + duration
    while time.monotonic() < end:
        set_pwm_servo(ser, 1, NEUTRAL)
        set_pwm_servo(ser, 2, NEUTRAL)
        time.sleep(0.05)

ser = serial.Serial(PORT, BAUD, timeout=1)
print('[OK] Serial port opened')
time.sleep(0.5)

# Bidirectional ESC mapping:
#   angle  0  = full speed direction A
#   angle 90  = stop (neutral)
#   angle 180 = full speed direction B

try:
    # Arm ESCs at minimum throttle (angle 0) for 3 seconds
    print('[ARM] Sending angle 0 for 3s...')
    for _ in range(60):
        set_pwm_servo(ser, 1, 0)
        set_pwm_servo(ser, 2, 0)
        time.sleep(0.05)
    print('[ARM] Done')
    time.sleep(1)

    # Test A: ramp upward from 0
    print('===== TEST A: Ramp up from 0 =====')
    for angle in [30, 50, 70, 90, 110]:
        print('[SPIN] angle={} for 2s...'.format(angle))
        for _ in range(40):
            set_pwm_servo(ser, 1, angle)
            set_pwm_servo(ser, 2, angle)
            time.sleep(0.05)

    # Stop
    print('[STOP] Back to angle 0 for 3s...')
    for _ in range(60):
        set_pwm_servo(ser, 1, 0)
        set_pwm_servo(ser, 2, 0)
        time.sleep(0.05)
    print('[STOP] Done')
    time.sleep(2)

    # Re-arm at angle 90, test B: ramp above 90
    print('===== TEST B: Arm at 90, ramp up =====')
    print('[ARM] Sending angle 90 for 3s...')
    for _ in range(60):
        set_pwm_servo(ser, 1, 90)
        set_pwm_servo(ser, 2, 90)
        time.sleep(0.05)
    print('[ARM] Done')
    time.sleep(1)

    for angle in [100, 120, 140, 160]:
        print('[SPIN] angle={} for 2s...'.format(angle))
        for _ in range(40):
            set_pwm_servo(ser, 1, angle)
            set_pwm_servo(ser, 2, angle)
            time.sleep(0.05)

    # Stop at 90
    print('[STOP] Back to angle 90 for 3s...')
    for _ in range(60):
        set_pwm_servo(ser, 1, 90)
        set_pwm_servo(ser, 2, 90)
        time.sleep(0.05)
    print('[STOP] Done')

finally:
    # Send both possible stop angles before closing
    print('[CLEANUP] Ensuring motors are stopped...')
    try:
        for _ in range(40):
            set_pwm_servo(ser, 1, 0)
            set_pwm_servo(ser, 2, 0)
            time.sleep(0.05)
        for _ in range(40):
            set_pwm_servo(ser, 1, 90)
            set_pwm_servo(ser, 2, 90)
            time.sleep(0.05)
    except Exception:
        pass
    ser.close()
    print('[DONE] Port closed')
    print()
    print('Which test made motors spin?')
    print('  Test A (arm at 0, ramp up)  => unidirectional ESC, stop = angle 0')
    print('  Test B (arm at 90, ramp up) => bidirectional ESC, stop = angle 90')

import serial
import time

# === Serial Setup ===
ARDUINO_PORT = '/dev/ttyACM1'  # Change if needed
BAUD_RATE = 9600
initial = 56
max_angle = 108
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
    print(f"Connected to {ARDUINO_PORT}")
except serial.SerialException as e:
    print(f"Error: Could not connect to {ARDUINO_PORT}: {e}")
    exit(1)

# === Send angle command to Arduino ===
def send_angle(value):
    angle = int(180 - float(value))
    #angle = int(max(56, min(108, angle)))  # Clamp between 56 and 108
    ser.write(f"{angle}\n".encode())
    print(f"Input slider value: {value} | Sent angle: {angle}")

try:
    # Simulate slider input from 124 to 72 (matches GUI behavior)
    for value in range(initial, max_angle):  # Decreasing order to test right movement
        send_angle(value)
        time.sleep(0.5)  # Allow servo to move

    # Then back from 72 to 124 to test left movement
    for value in range(max_angle, initial, -1):
        send_angle(value)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Simulation interrupted.")

finally:
    ser.close()
    print("Serial connection closed.")


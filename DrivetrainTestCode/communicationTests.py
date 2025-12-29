import serial
import struct
import time

# --- CONFIGURATION ---
PORT = '/dev/ttyUSB0'   # change to your port (COMx on Windows)
BAUDRATE = 115200
CMD_START = 0xAA        # must match Arduino definition
CMD_END   = 0xBB        # must match Arduino definition

# --- SETUP SERIAL ---
ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(2)  # wait for Arduino to reset

# --- USER INPUT ---
cmd_id = int(input("Enter command ID (integer): "))
num_params = int(input("Enter number of parameters: "))
params = []
for i in range(num_params):
    p = int(input(f"Enter parameter {i}: "))
    params.append(p)

# --- BUILD COMMAND ---
# Arduino expects: START_BYTE (1 byte), CMD_ID (int, 2 bytes), NUM_PARAMS (int, 2 bytes), PARAMS (int each, 2 bytes), END_BYTE (int, 2 bytes)
command_bytes = bytearray()

# Start byte
command_bytes.append(CMD_START)

# CMD_ID
command_bytes += struct.pack('<h', cmd_id)  # '<h' = little-endian 2-byte int

# NUM_PARAMS
command_bytes += struct.pack('<h', num_params)

# PARAMETERS
for p in params:
    command_bytes += struct.pack('<h', p)

# End byte
command_bytes += struct.pack('<h', CMD_END)

# --- SEND COMMAND ---
ser.write(command_bytes)
print("Sent command bytes:", list(command_bytes))

# --- READ ARDUINO DEBUG OUTPUT ---
print("\nArduino response:")
while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line:
        print(line)
    else:
        break  # stop if nothing comes in after timeout

ser.close()

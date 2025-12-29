import serial
import time

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
TIMEOUT = 0.2   # short timeout = "response finished"

CMD_START = 0xDEaD
CMD_END   = 0xBEEF

with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
    time.sleep(2)  # allow Arduino reset

    for cmd_id in range(256):
        params = []  # no params for now
        num_params = len(params)

        packet = bytes(
            [CMD_START, cmd_id, num_params] +
            params +
            [CMD_END]
        )

        # --- SEND COMMAND ---
        ser.write(packet)
        print(f"\nSent CMD {cmd_id:#04x}")

        # --- READ EVERYTHING RETURNED ---
        responses = []

        while True:
            line = ser.readline()
            if not line:
                break  # timeout → no more data

            try:
                decoded = line.decode("utf-8").strip()
                responses.append(decoded)
            except UnicodeDecodeError:
                responses.append(f"<binary> {line.hex()}")

        # --- PRINT RESPONSE ---
        for r in responses:
            print("Received:", r)

        # optional pacing
        time.sleep(1)

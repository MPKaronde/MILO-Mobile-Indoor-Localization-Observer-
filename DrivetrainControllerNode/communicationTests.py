import serial
import time

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
TIMEOUT = .8  # short timeout for "end of response"

with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
    time.sleep(2)

    for i in range(10):
        ser.write(f"{i}\n".encode())
        print(f"Sent: {i}")

        responses = []

        while True:
            line = ser.readline()
            if not line:
                break  # no more data -> end of response
            responses.append(line.decode(errors="replace").strip())

        for r in responses:
            print(f"Received: {r}")

        print("---")

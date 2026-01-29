import serial
import numpy as np

# overall workflow:
# The Python program sits in a loop, waits for one full frame
# to arrive from the Arduino,turns that frame from text into numbers,
# reshapes those numbers into a matrix, and then repeats forever.


# Open serial port (adjust port name if needed)
ser = serial.Serial("/dev/ttyACM0", 921600, timeout=1)

num_supply = None
num_sense = None

while True:
    line = ser.readline()  # bytes
    if not line:
        continue

    # Convert bytes → string
    s = line.decode("utf-8", errors="ignore").strip()
    if not s:
        continue

    parts = s.split(",")

    # Handle config line
    if parts[0] == "CFG":
        num_supply = int(parts[1])
        num_sense  = int(parts[2])
        print(f"Configured for {num_supply} × {num_sense}")
        continue

    # Handle frame line
    if parts[0] == "F" and num_supply is not None:
        timestamp_us = int(parts[1])

        # Convert remaining values to ints
        values = np.array(list(map(int, parts[2:])), dtype=np.int32)

        # Reshape into matrix
        frame = values.reshape((num_supply, num_sense))

        # Example: access a taxel
        print("Time:", timestamp_us,
              "Top-left taxel:", frame[0, 0])

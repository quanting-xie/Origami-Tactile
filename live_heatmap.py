import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

# Open serial port
ser = serial.Serial("/dev/cu.usbmodem21201", 921600, timeout=1)

num_supply = None
num_sense = None

# Wait for CFG line before setting up the plot
while num_supply is None:
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    parts = line.split(",")
    if parts[0] == "CFG":
        num_supply = int(parts[1])
        num_sense  = int(parts[2])
        print(f"Configured for {num_supply} × {num_sense}")

# Set up the plot once
fig, ax = plt.subplots()
placeholder = np.zeros((num_supply, num_sense))
# theoretically vmax should be 1023 but im gonna put it lower cuz im reading lower values
vmax = 1023
img = ax.imshow(placeholder, vmin=0, vmax=vmax, cmap="hot", interpolation="nearest")
plt.colorbar(img, ax=ax, label=f"ADC Value (0–{vmax})")
ax.set_title("FSR Array Live Heatmap")
ax.set_xlabel("Sense Column")
ax.set_ylabel("Supply Row")
plt.ion()   # interactive mode — allows live updates without blocking
plt.show()

# Main loop
while True:
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    if not line:
        continue

    parts = line.split(",")

    if parts[0] == "F" and len(parts) >= 2 + num_supply * num_sense:
        values = np.array(list(map(int, parts[2:])), dtype=np.int32)
        frame  = values.reshape((num_supply, num_sense))

        img.set_data(frame)   # update the heatmap data
        fig.canvas.flush_events()  # redraw
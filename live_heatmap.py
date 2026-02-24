import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

# Open serial port
ser = serial.Serial("/dev/cu.usbmodem21201", 921600, timeout=1)

num_rows_sup = None # number of rows
num_cols_sen = None # number of cols

# Wait for CFG line before setting up the plot
while num_rows_sup is None:
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    parts = line.split(",")
    if parts[0] == "CFG": # the first part of the config message printed to serail by arduino
        num_rows_sup = int(parts[1])
        num_cols_sen  = int(parts[2])
        print(f"Configured for {num_rows_sup} × {num_cols_sen}")



# Setting up the plot before taking in the voltage data
fig, ax = plt.subplots()
placeholder = np.zeros((num_rows_sup, num_cols_sen))
# theoretically vmax should be 1023 but im gonna put it lower cuz im reading lower values (nvm)
vmax = 1023 # max ADC value
img = ax.imshow(placeholder, vmin=0, vmax=vmax, cmap="hot", interpolation="nearest")
plt.colorbar(img, ax=ax, label=f"ADC Value (0–{vmax})")
ax.set_title("FSR Array Live Heatmap")
ax.set_xlabel("Sense Column")
ax.set_ylabel("Supply Row")
plt.ion()   # interactive mode — allows live updates without blocking
print('going to show the heatmap now!')
plt.show()

# Continoues main loop displaying new frames as they are sent by arduino through serial port
while True:
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    if line == "":
        continue

    parts = line.split(",")

    if parts[0] == "F" and len(parts) >= 2 + num_rows_sup * num_cols_sen:
        map_object = map(int, parts[2:])
        values = np.array(list(map_object), dtype=np.int32) # convert list of integer values to a np array
        frame  = values.reshape((num_rows_sup, num_cols_sen))

        img.set_data(frame)   # update the heatmap data
        fig.canvas.flush_events()  # redraw the figure
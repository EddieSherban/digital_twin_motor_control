import serial # pip install pyserial
import csv
import numpy as np
import matplotlib.pyplot as plt # pip install matplotlib
from matplotlib.animation import FuncAnimation

timestamps = []
duty_cycles = []
directions = []
velocities = []
velocities_ema = []
positions = []

prev_timestamp = None

frame_start = 0
timestamp = 0
duty_cycle = 0
direction = 0
velocity = 0
velocity_ema = 0
position = 0
frame_end = 0

def update_data(frame):
    global prev_timestamp

    try:
        data = port.readline().decode().strip()
        port.reset_input_buffer()

        frame_start, timestamp, duty_cycle, direction, velocity, velocity_ema, position, frame_end = map(float,data.split(','))
        if prev_timestamp is not None and timestamp < prev_timestamp:
            timestamps.clear()
            duty_cycles.clear()
            directions.clear()
            velocities.clear()
            velocities_ema.clear()
            positions.clear()
        if frame_start == 0x1 and frame_end == 0x3:
            timestamps.append(timestamp / 1000)
            duty_cycles.append(duty_cycle * 100)
            directions.append(direction)
            velocities.append(velocity)
            velocities_ema.append(velocity_ema)
            positions.append(position)

        prev_timestamp = timestamp

        # print(data)
        print(timestamp, np.mean(velocities_ema[-100:]), np.std(velocities_ema[-100:]), np.mean(velocities_ema[-100:]) - np.min(velocities_ema[-100:]), np.max(velocities_ema[-100:]) - np.mean(velocities_ema[-100:]))

        ax1.clear()
        ax2.clear()

        ax1.plot(timestamps, duty_cycles, 'r-', label='Duty Cycle')
        ax2.plot(timestamps, velocities, 'g-', label='Velocity (rad/s)')
        ax2.plot(timestamps, velocities_ema, 'b-', label='Velocity EMA (rad/s)')

        ax1.set_xlabel('Timestamp (s)')
        ax1.set_ylabel('Duty Cycle (%)')
        ax1.set_title('Real-Time Data')
        ax2.set_ylabel('Velocity (rad/s)')
        ax2.yaxis.set_label_position("right")
        ax1.set_ylim([-0.01 * 100, 1.01 * 100])
        ax2.set_ylim([-0.01 * 3.141592653, 1.01 * 3.141592653])
        ax2.legend()

    except Exception as e:
        print("Error:", e)

success = False
while not success:
    try:
        port = serial.Serial('COM9', 921600)
        success = True
    except Exception as e:
        print("Error:", e)

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

ani = FuncAnimation(fig, update_data, interval=10)
plt.show()

# with open('data.csv', 'w', newline='') as csv_file:
#     csv_writer = csv.writer(csv_file)
#     csv_writer.writerow(['Timestamp (ms)', 'Duty Cycle', 'Direction', 'Velocity (rad/s)', 'Position (rad)'])
#
#     while True:
#         try:
#             data = port.readline().decode().strip()
#             timestamp, duty_cycle, direction, velocity, position = data.split(',')
#             print(timestamp, duty_cycle, direction, velocity, position)
#             csv_writer.writerow([timestamp, duty_cycle, direction, velocity, position])
#         except Exception as e:
#             print("Error:", e)
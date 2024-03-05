import serial # pip install pyserial
import csv
import numpy as np
import matplotlib.pyplot as plt # pip install matplotlib
from matplotlib.animation import FuncAnimation
import threading

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

        frame_start, timestamp, duty_cycle, direction, velocity, velocity_ema, position, frame_end = map(float,data.split(','))

        if frame_start == 0x1 and frame_end == 0x3:
            timestamps.append(timestamp / 1000)
            duty_cycles.append(duty_cycle * 100)
            directions.append(direction)
            velocities.append(velocity)
            velocities_ema.append(velocity_ema)
            positions.append(position)
        if prev_timestamp is not None and timestamp < prev_timestamp:
            timestamps.clear()
            duty_cycles.clear()
            directions.clear()
            velocities.clear()
            velocities_ema.clear()
            positions.clear()

        prev_timestamp = timestamp

        # print(data)
        # print(
        #     "{:.0f} | Timestamp (s): {:.0f} Mean (RPM): {:.5f} Std Dev (RPM): {:.5f} Min Dev (RPM): {:.5f} Max Dev (RPM): {:.5f}".format(
        #         len(timestamps), timestamp, np.mean(velocities_ema[-500:]), np.std(velocities_ema[-500:]),
        #         np.mean(velocities_ema[-500:]) - np.min(velocities_ema[-500:]),
        #         np.max(velocities_ema[-500:]) - np.mean(velocities_ema[-500:])))

        ax1.clear()
        ax2.clear()

        ax1.plot(timestamps, duty_cycles, 'r-', label='Duty Cycle')
        ax2.plot(timestamps, velocities, 'g-', label='Velocity (RPM)')
        ax2.plot(timestamps, velocities_ema, 'b-', label='Velocity EMA (RPM)')
        ax1.text(1, 90, "Samples: " + str(len(timestamps)), size=10)
        ax1.text(1, 80, "Mean: " + str(np.mean(velocities_ema[-100:])), size=10)
        ax1.text(1, 70, "Min Dev: " + str(np.mean(velocities_ema[-100:]) - np.min(velocities_ema[-100:])), size=10)
        ax1.text(1, 60, "Max Dev: " + str(np.max(velocities_ema[-100:]) - np.mean(velocities_ema[-100:])), size=10)

        ax1.set_xlabel('Timestamp (s)')
        ax1.set_ylabel('Duty Cycle (%)')
        ax1.set_title('Real-Time Data')
        ax2.set_ylabel('Velocity (RPM)')

        ax2.yaxis.set_label_position("right")
        ax1.set_ylim([-0.01 * 100, 1.01 * 100])
        ax2.set_ylim([-0.01 * 30, 1.01 * 30])
        ax2.legend()

        port.reset_input_buffer()

    except Exception as e:
        pass
        # print("Error:", e)

success = False
while not success:
    try:
        port = serial.Serial('COM7', 921600)
        success = True
    except Exception as e:
        print("Error:", e)

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()
def send_data():
    while True:
        rx_data = input("Enter data to send: ")
        port.write(rx_data.encode())

rx_thread = threading.Thread(target=send_data)
rx_thread.start()

ani = FuncAnimation(fig, update_data, interval=7)

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
import serial # pip install pyserial
import csv
import numpy as np
import matplotlib.pyplot as plt # pip install matplotlib
from matplotlib.animation import FuncAnimation
import threading
import time

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

mean = 0
max_dev = 0
rx_data_event = threading.Event()
data = ""

def update_data(frame):
    global timestamp
    global duty_cycle
    global velocity
    global velocity_ema

    global prev_timestamp
    global mean
    global max_dev
    global data

    try:
        # port.reset_input_buffer()
        data = port.readline().decode().strip()

        frame_start, timestamp, duty_cycle, direction, velocity, velocity_ema, position, frame_end = map(float, data.split(','))
        if timestamp in timestamps:
            pass

        if frame_start == 0x1 and frame_end == 0x3:
            timestamps.append(timestamp)
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

        sample_size = 50
        prev_timestamp = timestamp
        mean = np.mean(velocities_ema[-sample_size:])
        max_dev = max(mean - np.min(velocities_ema[-sample_size:]), np.max(velocities_ema[-sample_size:]) - mean)

        ax1.clear()
        ax2.clear()

        ax1.plot(timestamps, duty_cycles, 'r-', label='Duty Cycle')
        ax2.plot(timestamps, velocities, 'g-', label='Velocity (RPM)')
        ax2.plot(timestamps, velocities_ema, 'b-', label='Velocity EMA (RPM)')

        window_time = 5000 # ms
        text_y = 90
        y1_lim = 100
        y2_lim = 30

        ax1.text(timestamp - window_time + 100, text_y, "Samples: " + str(len(timestamps)), size=10)
        ax1.text(timestamp - window_time + 100, text_y - 10, "Mean: " + str(mean), size=10)
        ax1.text(timestamp - window_time + 100, text_y - 20, "Max Dev.: " + str(max_dev), size=10)

        ax1.set_xlabel('Timestamp (ms)')
        ax1.set_ylabel('Duty Cycle (%)')
        ax1.set_title('Real-Time Data')
        ax2.set_ylabel('Velocity (RPM)')
        ax2.yaxis.set_label_position("right")

        ax1.set_xlim([timestamp - window_time, timestamp])
        ax1.set_ylim([-0.01 * y1_lim, 1.01 * y1_lim])
        ax2.set_ylim([-0.01 * y2_lim, 1.01 * y2_lim])
        ax2.legend()

    except Exception as e:
        print("Error:", e)

def send_data():
    global port
    while True:
        try:
            rx_data = ""
            rx_data = input("Enter data to send: ")
            if rx_data == 's':
                rx_data_event.set()
            elif rx_data == 'r':
                port.close()
                success = False
                while not success:
                    try:
                        port = serial.Serial('COM7', 921600)
                        success = True
                    except Exception as e:
                        print("Error:", e)
                port.reset_input_buffer()
                timestamps.clear()
                duty_cycles.clear()
                directions.clear()
                velocities.clear()
                velocities_ema.clear()
                positions.clear()
            else:
                port.write(rx_data.encode())
        except Exception as e:
            print("Error:", e)

def save_data():
    while True:
        # rx_data_event.wait()
        # try:
        #     with open('data.csv', 'a', newline='') as csv_file:
        #         csv_writer = csv.writer(csv_file)
        #         print('\n',[duty_cycle, 0, mean, max_dev],'\n')
        #         csv_writer.writerow([duty_cycle, 0, mean, max_dev])
        # except Exception as e:
        #     print("Error:", e)
        # rx_data_event.clear()
        try:
            with open('data.csv', 'a', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([timestamp, duty_cycle, velocity, velocity_ema])
        except Exception as e:
            print("Error:", e)

# Start serial communication
success = False
while not success:
    try:
        port = serial.Serial('COM7', 921600)
        success = True
    except Exception as e:
        print("Error:", e)


port.reset_input_buffer()

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

rx_thread = threading.Thread(target=send_data)
rx_thread.start()

# with open('data.csv', 'w', newline='') as csv_file:
#     csv_writer = csv.writer(csv_file)
#     # csv_writer.writerow(
#     #     ['Duty Cycle (%)', 'Reference Velocity (RPM)', 'Measured Velocity (RPM)', 'Max Deviation (RPM)'])
#     csv_writer.writerow(
#         ['Timestamp (ms)', 'Duty Cycle (%)', 'Velocity (RPM)', 'Velocity EMA (RPM)'])

# save_thread = threading.Thread(target=save_data)
# save_thread.start()

ani = FuncAnimation(fig, update_data, interval=1)

plt.show()
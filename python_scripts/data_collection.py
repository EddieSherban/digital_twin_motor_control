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

frame_start, timestamp, duty_cycle, direction, velocity, velocity_ema, position, frame_end = None

mean, = 0
 = 0
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
        data = port.readline().decode().strip()
        frame_start, timestamp, duty_cycle, velocity, velocity_ema, position, frame_end = map(float, data.split(','))

        if timestamp in timestamps:
            pass

        if frame_start == 0x1 and frame_end == 0x3:
            port.reset_input_buffer()
            timestamps.append(timestamp)
            duty_cycles.append(duty_cycle * 100)
            # directions.append(direction)
            velocities.append(velocity)
            velocities_ema.append(velocity_ema)
            positions.append(position)

        if prev_timestamp is not None and timestamp < prev_timestamp:
            timestamps.clear()
            duty_cycles.clear()
            # directions.clear()
            velocities.clear()
            velocities_ema.clear()
            positions.clear()

        sample_size = 50
        prev_timestamp = timestamp
        raw_mean = np.mean(velocities[-sample_size:])
        ema_mean = np.mean(velocities_ema[-sample_size:])
        raw_dev = max(mean - np.min(velocities[-sample_size:]), np.max(velocities[-sample_size:]) - raw_mean)
        ema_dev = max(mean - np.min(velocities[-sample_size:]), np.max(velocities_ema[-sample_size:]) - ema_mean)

        ax1.clear()
        ax2.clear()

        ax1.plot(timestamps, duty_cycles, 'r-', label='Duty Cycle')
        # ax1.plot(timestamps, positions, 'o-', label='Position (Deg)')
        ax2.plot(timestamps, velocities, 'y-', label='Velocity (RPM)')
        ax2.plot(timestamps, velocities_ema, 'g-', label='Velocity EMA (RPM)')

        window_time = 60000 # ms
        y1_lim = 360
        y2_lim = 30
        text_y = y1_lim * 0.9

        ax1.text(timestamp - window_time + 1000, text_y, "Samples: " + str(len(timestamps)), size=10)
        ax1.text(timestamp - window_time + 1000, text_y - y1_lim * 0.1, "Raw Mean: " + str(raw_mean), size=10)
        ax1.text(timestamp - window_time + 1000, text_y - y1_lim * 0.2, "EMA Mean: " + str(ema_mean), size=10)
        ax1.text(timestamp - window_time + 1000, text_y - y1_lim * 0.3, "Raw Dev.: " + str(raw_dev), size=10)
        ax1.text(timestamp - window_time + 1000, text_y - y1_lim * 0.4, "EMA Dev.: " + str(ema_dev), size=10)
        ax1.set_title('Real-Time Data')
        ax1.set_xlabel('Timestamp (ms)')
        ax1.set_ylabel('Position (Deg)')
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
        port.reset_input_buffer()
        success = True
    except Exception as e:
        print("Error:", e)

# with open('data.csv', 'w', newline='') as csv_file:
#     csv_writer = csv.writer(csv_file)
#     csv_writer.writerow(['Timestamp (ms)', 'Duty Cycle (%)', 'Velocity (RPM)', 'Velocity EMA (N=5) (RPM)'])
#     while True:
#         try:
#             data = port.readline().decode().strip()
#             frame_start, timestamp, duty_cycle, direction, velocity, velocity_ema, position, frame_end = map(float,data.split(','))
#
#             if frame_start == 0x1 and frame_end == 0x3:
#                 csv_writer = csv.writer(csv_file)
#                 csv_writer.writerow([timestamp, duty_cycle, velocity, velocity_ema])
#                 if timestamp > 15000:
#                     port.close()
#                     exit()
#         except Exception as e:
#             print("Error:", e)



# port.reset_input_buffer()

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

rx_thread = threading.Thread(target=send_data)
rx_thread.start()

# save_thread = threading.Thread(target=save_data)
# save_thread.start()

ani = FuncAnimation(fig, update_data, interval=1)

plt.show()
import serial           # pip install pyserial
import matplotlib.pyplot as plt # pip install matplotlib
from matplotlib.animation import FuncAnimation
import numpy as np
import threading
from scipy.interpolate import interp1d
import time

# Constants
COMM_PORT = 'COM7'
BAUD_RATE = 1000000

START_TIME = 1000
SAMPLE_SIZE = 100
SAMPLE_TIME = 60000

Y1_LIMIT = 100
Y2_LIMIT = 30
Y3_LIMIT = 360

timestamp_list = np.empty(0)
duty_cycle_list = np.empty(0)
velocity_list = np.empty(0)
velocity_ema_list = np.empty(0)
position_list = np.empty(0)

interpolated_timestamp = np.empty(0)
interpolated_velocity_ema = np.empty(0)
interpolated_position = np.empty(0)

def update_graph(frame):
    global timestamp_list, duty_cycle_list, velocity_list, velocity_ema_list, position_list

    while True:
        try:
            data = comm.readline().decode()
            if data[0] == '1' and data[-2] == '3':
                if len(data.split(',')) == 7:
                    comm.reset_input_buffer()
                    break
        except Exception as e:
            print("Error:", e)

    try:
        [frame_start, timestamp, duty_cycle, velocity, velocity_ema, position, frame_end] = map(float, data.split(','))

        if timestamp < START_TIME:
            timestamp_list = np.empty(0)
            duty_cycle_list = np.empty(0)
            velocity_list = np.empty(0)
            velocity_ema_list = np.empty(0)
            position_list = np.empty(0)

        timestamp_list = np.append(timestamp_list, timestamp)
        duty_cycle_list = np.append(duty_cycle_list, duty_cycle * 100)
        velocity_list = np.append(velocity_list, velocity)
        velocity_ema_list = np.append(velocity_ema_list, velocity_ema)
        position_list = np.append(position_list, position)

        mean = np.mean(velocity_ema_list[-SAMPLE_SIZE:])
        dev = max(mean - np.min(velocity_ema_list[-SAMPLE_SIZE:]), np.max(velocity_ema_list[-SAMPLE_SIZE:]) - mean)

        ax1.clear()
        ax2.clear()
        ax3.clear()

        ax1.plot(timestamp_list, duty_cycle_list, 'r-', label='Duty Cycle')
        ax2.plot(timestamp_list, velocity_ema_list, 'g-', label='Velocity (RPM)')
        ax3.plot(timestamp_list, position_list, 'b-', label='Position (Deg)')

        ax1.text(timestamp_list[-1] - SAMPLE_TIME + SAMPLE_TIME * 0.1, Y1_LIMIT * 0.9 - Y1_LIMIT * 0.10,
                 "Samples: " + str(len(timestamp_list)), size=10)
        ax1.text(timestamp_list[-1] - SAMPLE_TIME + SAMPLE_TIME * 0.1, Y1_LIMIT * 0.9 - Y1_LIMIT * 0.15,
                 "Mean: " + str(mean),
                 size=10)
        ax1.text(timestamp_list[-1] - SAMPLE_TIME + SAMPLE_TIME * 0.1, Y1_LIMIT * 0.9 - Y1_LIMIT * 0.20,
                 "Dev: " + str(dev),
                 size=10)

        ax1.set_title('Real-Time Data')
        ax1.set_xlabel('Timestamp (ms)')
        ax1.set_ylabel('Duty Cycle (%)', color='r')
        ax2.set_ylabel('Velocity (RPM)', color='g')
        ax3.set_ylabel('Position (Deg)', color='b')


        ax2.spines['right'].set_position(('outward', 0))
        ax3.spines['right'].set_position(('outward', 40))
        ax2.yaxis.set_label_position("right")
        ax3.yaxis.set_label_position("right")

        ax1.set_xlim([timestamp_list[-1] - SAMPLE_TIME, timestamp_list[-1]])
        ax1.set_ylim([-0.01 * Y1_LIMIT, 1.01 * Y1_LIMIT])
        ax2.set_ylim([-0.01 * Y2_LIMIT, 1.01 * Y2_LIMIT])
        ax3.set_ylim([-0.01 * Y3_LIMIT, 1.01 * Y3_LIMIT])

        handles1, labels1 = ax1.get_legend_handles_labels()
        handles2, labels2 = ax2.get_legend_handles_labels()
        handles3, labels3 = ax3.get_legend_handles_labels()

        handles = handles1 + handles2 + handles3
        labels = labels1 + labels2 + labels3
        ax2.legend(handles, labels, loc='upper right')

    except Exception as e:
        print("Error:", e)



def interpolate(frame):
    global interpolated_timestamp, interpolated_velocity_ema, interpolated_position

    try:
        max_length = max(len(timestamp_list), len(velocity_ema_list), len(position_list))
        padded_timestamp = np.pad(timestamp_list, (0, max_length - len(timestamp_list)), mode='constant')
        padded_velocity_ema = np.pad(velocity_ema_list, (0, max_length - len(velocity_ema_list)), mode='constant')
        padded_position = np.pad(position_list, (0, max_length - len(position_list)), mode='constant')
        inter_velocity_ema = interp1d(padded_timestamp, padded_velocity_ema, kind='cubic')
        inter_position = interp1d(padded_timestamp, padded_position, kind='cubic')

        interpolated_timestamp = np.linspace(timestamp_list[0], timestamp_list[-1], num=len(timestamp_list))
        interpolated_velocity_ema = inter_velocity_ema(interpolated_timestamp)
        interpolated_position = inter_position(interpolated_timestamp)

        ax1_interpolated.clear()
        ax2_interpolated.clear()

        ax1_interpolated.plot(interpolated_timestamp, interpolated_position, 'g-.', label='Position (Deg)')
        ax2_interpolated.plot(interpolated_timestamp, interpolated_velocity_ema, 'b-.', label='Velocity (RPM)')

        ax1_interpolated.set_title('Real-Time Data')
        ax1_interpolated.set_xlabel('Timestamp (ms)')
        ax1_interpolated.set_ylabel('Position (Deg)')
        ax2_interpolated.set_ylabel('Velocity (RPM)')
        ax2_interpolated.yaxis.set_label_position("right")

        ax1_interpolated.set_xlim([timestamp_list[-1] - SAMPLE_TIME, timestamp_list[-1]])
        ax1_interpolated.set_ylim([-0.01 * Y1_LIMIT, 1.01 * Y1_LIMIT])
        ax2_interpolated.set_ylim([-0.01 * Y2_LIMIT, 1.01 * Y2_LIMIT])
        ax2_interpolated.legend()

    except Exception as e:
        print("Error:", e)

connected = False
while not connected:
    try:
        comm = serial.Serial(COMM_PORT, BAUD_RATE)
        connected = True
    except:
        pass
comm.reset_input_buffer()

fig, ax1 = plt.subplots(figsize=(15, 10))
ax2 = ax1.twinx()
ax3 = ax1.twinx()
ani = FuncAnimation(fig, update_graph, interval=10)

# fig_interpolated, ax1_interpolated = plt.subplots()
# ax2_interpolated = ax1_interpolated.twinx()
# ani_interpolated = FuncAnimation(fig_interpolated, interpolate, interval=50)

plt.show()





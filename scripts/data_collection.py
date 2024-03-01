import serial # pip install pyserial
import csv
import numpy as np
import matplotlib.pyplot as plt # pip install matplotlib
from matplotlib.animation import FuncAnimation

timestamps = []
duty_cycles = []
directions = []
velocities = []
positions = []

def update_data(frame):
    try:
        data = port.readline().decode().strip()
        if len(data.split(',')) == 5:
            timestamp, duty_cycle, direction, velocity, position = map(float, data.split(','))
            timestamps.append(timestamp)
            duty_cycles.append(duty_cycle)
            #directions.append(direction)
            velocities.append(velocity)
            #positions.append(position)

        #print(timestamp, duty_cycle, direction, velocity, position)
        print(timestamp, np.mean(velocities), np.std(velocities))

        ax.clear()
        #ax.plot(timestamps, directions, label='Direction')
        ax.plot(timestamps, duty_cycles, label='Duty Cycle')
        ax.plot(timestamps, velocities, label='Velocity (rad/s)')
        #ax.plot(timestamps, positions, label='Position (rad)')
        ax.legend()
    except Exception as e:
        print("Error:", e)

success = False
while not success:
    try:
        port = serial.Serial('COM7', 115200)
        success = True
    except Exception as e:
        print("Error:", e)
        timestamps = []
        duty_cycles = []
        directions = []
        velocities = []
        positions = []

fig, ax = plt.subplots()
ax.set_xlabel('Timestamp (ms)')
ax.set_ylabel('Value')
ax.set_title('Real-Time Data')

ani = FuncAnimation(fig, update_data, interval=1)
plt.show()

with open('data.csv', 'w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['Timestamp (ms)', 'Duty Cycle', 'Direction', 'Velocity (rad/s)', 'Position (rad)'])

    while True:
        try:
            data = port.readline().decode().strip()
            timestamp, duty_cycle, direction, velocity, position = data.split(',')
            print(timestamp, duty_cycle, direction, velocity, position)
            csv_writer.writerow([timestamp, duty_cycle, direction, velocity, position])
        except Exception as e:
            print("Error:", e)
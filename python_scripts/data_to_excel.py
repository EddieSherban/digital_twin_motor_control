import time

import serial           # pip install pyserial
import pandas as pd     # pip install pandas
import datetime
import json

# Constants
COMM_PORT = 'COM7'
BAUD_RATE = 921600
FILE_NAME = 'data'

SAMPLE_INTERVAL = 30

current_time = datetime.datetime.now().strftime('%B-%d_%Hh%Mm%Ss')
FILE_NAME += '_' + current_time + '.xlsx'

timestamp = []
direction = []
duty_cycle = []
velocity = []
position = []
current = []

command = input('Pause...')

connected = False
while not connected:
    try:
        comm = serial.Serial(
            COMM_PORT,
            BAUD_RATE,
        )
        connected = True
    except:
        pass
comm.reset_input_buffer()

df = pd.DataFrame()
sample_count = 0
while True:
    try:
        json_string = comm.readline().decode().strip()
        data = json.loads(json_string)

        for key, values in data.items():
            if key == "timestamp":
                timestamp.extend(values)
            elif key == "direction":
                direction.extend(values)
            elif key == "duty_cycle":
                duty_cycle.extend(values)
            elif key == "velocity":
                velocity.extend(values)
            elif key == "position":
                position.extend(values)
            elif key == "current":
                current.extend(values)

        print("Timestamp:", timestamp)
        print("Direction:", direction)
        print("Duty Cycle:", duty_cycle)
        print("Velocity:", velocity)
        print("Position:", position)
        print("Current:", current)

        sample_count += 1
        print(sample_count)

        if sample_count >= SAMPLE_INTERVAL:
            data = {'Timestamp': timestamp,
                    'Direction': direction,
                    'Duty Cycle': duty_cycle,
                    'Velocity (RPM)': velocity,
                    'Position (Deg)': position,
                    'Current (mA)': current}
            df = pd.DataFrame(data)
            df.to_excel(FILE_NAME, index=False)
            exit()
    except Exception as e:
        #comm.reset_input_buffer()
        print("Error:", e)




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
        rx_data = comm.readline()
        print(rx_data)
        print("\n")
        rx_data = rx_data.decode().strip()
        print(rx_data)

        data_list = json.loads(rx_data)

        timestamp += data_list[0]
        direction += data_list[1]
        duty_cycle += data_list[2]
        velocity += data_list[3]
        position += data_list[4]
        current += data_list[5]

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




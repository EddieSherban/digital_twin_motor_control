import time

import serial           # pip install pyserial
import pandas as pd     # pip install pandas
import datetime

# Constants
COMM_PORT = 'COM7'
BAUD_RATE = 921600
FILE_NAME = 'data'

START_TIME = 3000
END_TIME = 180000
SAMPLE_INTERVAL = 600000

current_time = datetime.datetime.now().strftime('%B-%d_%Hh%Mm%Ss')
FILE_NAME += '_' + current_time + '.xlsx'

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
        frame = comm.readline()
        if frame[0] == 0x1 and frame[-2] == 0x3:
            data = frame.decode().replace(',', '').replace(', ', '')
            [timestamp, direction, duty_cycle, velocity, position, current] = map(float, data.split(','))
            sample = pd.DataFrame({
                'Timestamp (ms)': [timestamp],
                'Duty Cycle': [duty_cycle],
                'Velocity (RPM)': [velocity],
                'Position (Deg)': [position],
                'Current (mA)': [current],
            })
            df = pd.concat([df, sample], ignore_index=True)
            sample_count += 1
            print(data, sample_count)
            if sample_count >= SAMPLE_INTERVAL:
                df.to_excel(FILE_NAME, index=False)
                exit()
    except Exception as e:
        comm.reset_input_buffer()
        print("Error:", e)




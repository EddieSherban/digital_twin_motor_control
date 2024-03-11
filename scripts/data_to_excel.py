import serial           # pip install pyserial
import pandas as pd     # pip install pandas
import datetime

# Constants
COMM_PORT = 'COM7'
BAUD_RATE = 1000000
FILE_NAME = 'data'

START_TIME = 4000
SAMPLE_INTERVAL = 5000

current_time = datetime.datetime.now().strftime('%B-%d_%Hh%Mm%Ss')
FILE_NAME += '_' + current_time + '.xlsx'

command = input('Pause...')

connected = False
while not connected:
    try:
        comm = serial.Serial(COMM_PORT, BAUD_RATE)
        connected = True
    except:
        pass
comm.reset_input_buffer()

df = pd.DataFrame()
sample_count = 0
while True:
    try:
        while True:
            try:
                data = comm.readline().decode()
                if data[0] == '1' and data[-2] == '3':
                    if len(data.split(',')) == 7:
                        break
            except Exception as e:
                print("Error:", e)

        [frame_start, timestamp, duty_cycle, velocity, velocity_ema, position, frame_end] = map(float, data.split(','))

        if frame_start == 0x1 and frame_end == 0x3:
            frame_start = frame_end = None
            if timestamp < START_TIME:
                df = pd.DataFrame()
                sample_count = 0
            sample = pd.DataFrame({
                'Timestamp (ms)': [timestamp],
                'Duty Cycle': [duty_cycle],
                'Velocity (RPM)': [velocity],
                'Velocity EMA (RPM)': [velocity_ema],
                'Position (Deg)': [position],
            })
            df = pd.concat([df, sample], ignore_index=True)
            sample_count += 1
            if sample_count >= SAMPLE_INTERVAL:
                df.to_excel(FILE_NAME, index=False)
                command = input('Command: ')
                sample_count = 0
    except Exception as e:
        print("Error:", e)




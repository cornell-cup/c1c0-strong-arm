import serial
import time
import sys
import keyboard as kb


sys.path.append('/home/c1c0-main/c1c0-movement/c1c0-movement/Locomotion')
#Resolve 'cornellcup' to be the username of this Jetson, make sure c1c0-movement is cloned
import R2Protocol2 as r2p

ser = serial.Serial(
    port = '/dev/ttyTHS1', # Jetson hardware serial port (pins 8/10 on header)
    baudrate = 9600
)

data = [3, 3, 3, 3] # data[elbow, spin, hand, shoulder]
# Elbow:    3-stop,  1-in,    2-out
# Spin:     3-stop,  1-CW,    2-CCW
# Hand:     3-stop,  1-close, 2-open
# Shoulder: 3-stop,  1-up,    2-down

def convert_8_to_16(msg, length):
    data = []
    for i in range(0,length,2):
        data.append((msg[i] << 8) | msg[i+1])
    return data

def convert_16_to_8(msg, length):
    data = []
    for i in range(0,length):
        data.append((msg[i] >> 8) & 255)
        data.append(msg[i] & 255)
    return data

while(True):
    msg = r2p.encode(b"PRM", bytes(convert_16_to_8(data,4))) 
    ser.write(msg) #Message length will be 24 bytes (See r2p.encode() specification ) 16 header + 8 data bytes

    # Elbow control (w/g)
    if kb.is_pressed('w'): data[0] = 1    # Move elbow in
    elif kb.is_pressed('s'): data[0] = 2  # Move elbow out
    else: data[0] = 3

    # Wrist control (e/d)
    if kb.is_pressed('e'): data[1] = 1    # Spin wrist CW      
    elif kb.is_pressed('d'): data[1] = 2  # Spin wrist CCW
    else: data[1] = 3

    # Hand control (r/f)
    if kb.is_pressed('r'): data[2] = 1    # Close hand
    elif kb.is_pressed('f'): data[2] = 2  # Open hand
    else: data[2] = 3

    # Shoulder control (q/a)
    if kb.is_pressed('q'): data[3] = 1    # Move shoulder up
    elif kb.is_pressed('a'): data[3] = 2  # Move shoulder down
    else: data[3] = 3
    
    print("Sent message: " + str(data))
    time.sleep(.2)


    
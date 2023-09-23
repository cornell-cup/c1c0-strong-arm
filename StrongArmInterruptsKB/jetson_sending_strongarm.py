import serial
import time
import sys

x = 1

sys.path.append('/home/c1c0-main/c1c0-movement/c1c0-movement/Locomotion')
#Resolve 'cornellcup' to be the username of this Jetson, make sure c1c0-movement is cloned
import R2Protocol2 as r2p

ser = serial.Serial(
    port = '/dev/ttyTHS1', #Jetson hardware serial port (pins 8/10 on header)
    baudrate = 38400
)

data = [100, 120, 30, 1, 0, 0] #data[elbow,bend,spin,hand,?,?]

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

while(x==1):
    msg = r2p.encode(b"PRM", bytes(convert_16_to_8(data,6))) 

    ser.write(msg) #Message length will be 24 bytes (See r2p.encode() specification ) 16 header + 8 data bytes
    
    print("Sent message")
    time.sleep(1)
    x = 0

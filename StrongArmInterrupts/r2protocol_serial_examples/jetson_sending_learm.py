# Example File for Jetson Sending R2Protocol Messages to the Arduino
# In coordination with arduino_receiving.ino
# For use with c1c0 communication systems
# See the FAQ on the Cornell Cup Google Drive, located Here: 
# https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit

import serial
import time
import sys

sys.path.append('/home/cornellcup/c1c0-movement/c1c0-movement/Locomotion')
#Resolve 'cornellcup' to be the username of this Jetson, make sure c1c0-movement is cloned
import R2Protocol2 as r2p

ser = serial.Serial(
    port = '/dev/ttyTHS1', #Jetson hardware serial port (pins 8/10 on header)
    baudrate = 9600, #Bits/s data rate
)

data = [90,90,70,45,45,120]
init = 1

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

    if(init):
        ser_msg = ser.read(28)
        mtype,r_data,status = r2p.decode(ser_msg)
        print(convert_8_to_16(r_data,12))
        init = 0
        
    msg = r2p.encode(b"PRM", bytes(convert_16_to_8(data,6)))

    ser.write(msg) #Message length will be 24 bytes (See r2p.encode() specification ) 16 header + 8 data bytes
    
    print("Sent message")
    time.sleep(1)

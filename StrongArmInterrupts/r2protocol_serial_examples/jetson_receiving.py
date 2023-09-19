# Example File for Jetson Receiving R2Protocol Messages from the Arduino
# In coordination with arduino_sending.ino
# For use with c1c0 communication systems
# See the FAQ on the Cornell Cup Google Drive, located Here: 
# https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit


import serial
import time
import sys

sys.path.append('/home/cornellcup/c1c0-movement/c1c0-movement/Locomotion')
import R2Protocol2 as r2p

ser = serial.Serial(
    port = '/dev/ttyTHS1', #Jetson hardware serial port (pins 8/10 on header)
    baudrate = 115200, #Bits/s data rate
)

ser.reset_input_buffer() #Improve reliability, clear serial buffer
while(True):
    msg = ser.read_until(expected = b'\xd2\xe2\xf2') #Tail bits of r2p encoded message, see r2protocol.encode() specification

    msg_decoded = r2p.decode(msg)
    print(msg_decoded)

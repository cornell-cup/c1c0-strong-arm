import serial
import sys
import codecs

'''
Arduino to Jetson Communication with R2 Protocol
'''

sys.path.insert(0, './c1c0-movement/Locomotion/')
import R2Protocol2 as r2p
file1 = open("file1.txt", "w")

ser = None
 
def init_serial(port,baud):
	'''
	Initializes the serial port, usually set baud to 9600
	'''
	global ser, startseq, endseq
	ser = serial.Serial(port,baud)
	
def close_serial():
	'''
	Closes the serial port.
	'''
	global ser
	ser.close()
	
def read_encoder_values():
	'''
	Returns 3 encoder values i(n decimal) as an array.
	If the value is 1452, then the encoder is not powered or there is 
	a wiring issue. 
	'''
	global ser
	# initialize the array 
	encoderAngle = [0,0,0,0,0,0]
	try:
		while True:
			good_data = False
			while (not good_data):
				# read the serial port
				ser_msg = ser.read(28)#_until(b"\xa2\xb2\xc2")
				#ser_msg += ser.read(19)
				#print(ser_msg)
				# decode the serial message
				msgtype, msg, status = r2p.decode(ser_msg)
				#print(msg)
				# if the checksum is correct (1), then exit the while loop
				if (status):
					good_data = True
				# if the checksum is wrong, reset the buffer and try again
				else:
					#ser.reset_input_buffer()
					print('bad data')
			# convert array of length 6 to array of length 3		
			for i in range(0, 12, 2):
				encoderAngle[i//2] = (msg[i]<<8) | msg[i+1]
			print(encoderAngle)
		
	except KeyboardInterrupt: 		
			for i in range(0, 12, 2):
            			ser.close()
	
	
if __name__ =='__main__':
	init_serial('/dev/ttyTHS1', 38400)
	read_encoder_values()

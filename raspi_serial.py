import serial
import string
import time

try:
	ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1) # open serial port
except:
	ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1) # open serial port
ser.reset_input_buffer() # clear the buffers

# All Helper Functions!
def writeArduino(command):
	ser.write(command.encode())
	
def readArduino():
	return ser.readline().decode('utf-8').strip()

def get_US(sensor):
	command = "get_US," + sensor + "\n"
	ser.write(command.encode())
	return readArduino()

def buzz():
	command = "buzz\n"
	ser.write(command.encode())
	time.sleep(0.3)

while True:
	distance = float(get_US("FRONT_LEFT"))
	print(distance)
	if (distance > 10):
		buzz();
	if (ser.in_waiting > 0):
		print(readArduino())
	time.sleep(0.1)
	

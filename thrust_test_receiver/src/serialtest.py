#!/usr/bin/env python

# This is a simple test to see if the communication between the host (Udoo Bolt) and the built-in Arduino Leonardo works.
# On the Arduino, there is a simple PassThrough program running.

import serial
import time
import binascii 
import struct

def initserial():
	ser = serial.Serial(
	    port='/dev/ttyACM0',\
	    baudrate=115200,\
	    parity=serial.PARITY_NONE,\
	    stopbits=serial.STOPBITS_ONE,\
	    bytesize=serial.EIGHTBITS,\
	    timeout=0.1, \
	    xonxoff=False, rtscts=True, write_timeout=0.1, dsrdtr=True, inter_byte_timeout=None, exclusive=True)
	return ser

def to_bytes(int_16):
	b1 = int_16 >> 8
	b2 = int_16 & 0xff
	return (b1,b2)


if __name__ == "__main__":
	print("start")
	throttle_48 = to_bytes(48)
	throttle = to_bytes(123)
	print(throttle)
	
	arduinoserial = initserial()
	arduinoserial.write(throttle_48)

	while(True):
		print("current time: " + str(time.time()))
		print("write in binary")
		arduinoserial.write(to_bytes(0xffff))
		arduinoserial.write(throttle)
		read = arduinoserial.read(2)		# read two bytes
		# print(type(read)) 				# in python 2.7, the type is <str>
		print("received in hex: " + binascii.hexlify(read))
		print("received as int: " + str(struct.unpack('>H', read)[0]))
		# print(type(struct.unpack('>H', read)[0]))
		# print(arduinoserial.in_waiting, arduinoserial.out_waiting, arduinoserial.get_settings())


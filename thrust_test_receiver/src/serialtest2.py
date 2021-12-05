#!/usr/bin/env python
# -*- coding: latin_1 -*-

# This is a simple test to see if the communication between the host (Udoo Bolt) and the built-in Arduino Leonardo works.
# On the Arduino, there is the arduino_Serialtest.ino program running.

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
	# ser.reset_input_buffer()
	# ser.reset_output_buffer()
	return ser

def to_bytes(int_16):
	b1 = int_16 >> 8
	b2 = int_16 & 0xff
	return (b1,b2)


if __name__ == "__main__":
	throttle_48 = to_bytes(48)
	throttle = to_bytes(0xffff)
	print(throttle)
	
	arduinoserial = initserial()

	print(arduinoserial.in_waiting, arduinoserial.out_waiting, arduinoserial.get_settings())
	# arduinoserial.write(to_bytes(0xFFFF))
	# arduinoserial.write(throttle)

	while(True):
		print("current time: " + str(time.time()))
		# print(arduinoserial.in_waiting, arduinoserial.out_waiting, arduinoserial.get_settings())
		# print("write int in binary")
		# arduinoserial.write('\x39\xf7')
		# arduinoserial.write(throttle)
		t1 = unichr(throttle[0]).encode('latin_1')
		t2 = unichr(throttle[1]).encode('latin_1')
		print(arduinoserial.write(unichr(throttle[0]).encode('latin_1')))	# python doesn't want to send values over 0x7f (128) without the encoding
		time.sleep(0.1)
		print(arduinoserial.write(unichr(throttle[1]).encode('latin_1')))	# (https://stackoverflow.com/questions/14454957/pyserial-formatting-bytes-over-127-return-as-2-bytes-rather-then-one)

		# print(arduinoserial.in_waiting, arduinoserial.out_waiting, arduinoserial.get_settings())
		read = arduinoserial.read_until('\n')		# read line
		# print(type(read)) 				# in python 2.7, the type is <str>
		print("received from Arduino: " + read)
		time.sleep(0.1)


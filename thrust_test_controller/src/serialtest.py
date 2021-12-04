import serial
import time
import struct


def initserial():
    ser = serial.Serial(baudrate=115200, port='/dev/ttyACM0')
    ser.write(b'48')
    return ser

if __name__ == "__main__":
    print("start")
    throttle_0 = b'48'
    # throttle_2047 = (2047).to_bytes(2, "little")# or big
    # throttle_2047 =struct.pack('B', 2047)
    throttle_2047 = str((34,))
    i = 2047
    print(type(throttle_0))
    print(type(throttle_2047))
    print(throttle_0)
    print(throttle_2047)
    
    print(chr(2000))
    print(struct.pack('<i', 5)) # b'\x05\x00\x00\x00')

    print(b'\x34')

    arduinoserial = initserial()
    arduinoserial.write(throttle_0)


    while(True):
        arduinoserial.write(throttle_2047)
        wait(0.1)
        print(arduinoserial.readline())


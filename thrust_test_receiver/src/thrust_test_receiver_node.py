#!/usr/bin/env python

# Available control modes:
# 0: Individually control prop speeds
# 1: Both prop speeds equal
# 2: Ramp up, according to a specified lower and upper speed limit and a given ramp up time
# If PI control is enabled, the force and torque measurements will be used to adapt the lower rotor (i.e. rotor 1)
# so that the total net torque is zero. This is done via a PI controller.
# This is only valid for modes 1 and 2 (i.e. when both props are controlled to run at the same speed or for ramp up)

import rospy
import numpy as np
import serial
import struct
import crc8     # pip install crc8

# from dynamic_reconfigure.server import Server
# from thrust_test_controller.cfg import thrust_testConfig
from std_msgs.msg import UInt16 as cmd_msg
from mavros_msgs.msg import ESCTelemetryItem as esc_tlm_msg
# from omav_hovery_msgs.msg import UAVStatus as cmd_msg
# from omav_hovery_msgs.msg import MotorStatus
# from mavros_msgs.msg import TiltrotorActuatorCommands as cmd_msg
# from geometry_msgs.msg import WrenchStamped as wrench_msg

class ArduinoSerial():
    def __init__(self):
        self.serial = self.initserial()

    def write(self, int_16):
        # self.serial.write(self.to_bytes(int_16))
        self.serial.write(hex(int_16)[2:] + '\n')       # writes as hex string

    def read_int(self):
        b = self.serial.read(2)
        return self.to_int(b)

    def read_str(self):
        return self.serial.read_until(b'\n')

    def initserial(self):
        ser = serial.Serial(
            port='/dev/ttyACM0',\
            baudrate=115200,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
            timeout=0.1, \
            xonxoff=False, rtscts=True, write_timeout=0.1, dsrdtr=True, inter_byte_timeout=None, exclusive=True)
        return ser

    def printstats(self):
        print(self.serial.in_waiting, self.serial.out_waiting, self.serial.get_settings())

    @staticmethod
    def to_bytes(int_16):
        b1 = int_16 >> 8
        b2 = int_16 & 0xff
        return (b1,b2)

    @staticmethod
    def to_int(str_bytes):
        return struct.unpack('>H', str_bytes)[0]

class ESCNode():
    def __init__(self):
        self.throttle = 0
        self.last_command_time = rospy.get_rostime()
        self.control_mode = 0   # change to 0? (disarmed)
        topic = rospy.get_param('~topic', 'arduino/esc')
        rospy.loginfo('topic = %s', topic)
        self.rate_main = rospy.get_param('~rate', 5000.0)
        self.rate = rospy.Rate(self.rate_main)
        self.serial_rate = 1000
        self.serial_ticks = 0
        self.new_msg = True
        self.tlm_current = 0

        msg = cmd_msg()
        pub = rospy.Publisher(topic, cmd_msg, queue_size=10)
        topic = rospy.get_param('~topic', 'arduino/esc/tlm')

        tlm_msg = esc_tlm_msg()
        tlm_pub = rospy.Publisher('arduino/esc/tlm', esc_tlm_msg, queue_size=10)

        rospy.Subscriber('mavros/setpoint_raw/actuator_command', cmd_msg, self.set_esc_callback)

        # rospy.Subscriber("/rokubimini/ft_sensor0/ft_sensor_readings/wrench",
        #                  wrench_msg, self.rokubiCallback0)
        # rospy.Subscriber("/rokubimini/ft_sensor1/ft_sensor_readings/wrench",
        #                  wrench_msg, self.rokubiCallback1)
        # Wait until sensors ready
        # while(not(self.ft0Ready and self.ft1Ready)):
        #     rospy.sleep(1)
        # Remove constant bias
        # self.removeBias()

        rospy.loginfo(' Initializing both serial')
        self.ser = serial.Serial()
        self.ser.baudrate = rospy.get_param('~tlm_baud','115200')
        self.ser.port = rospy.get_param('~tlm_port','/dev/ttyS0') #'/dev/ttyUSB0')
        self.ser.timeout = 0.1
        if not self.ser.is_open:
            self.ser.open()
        self.ser.reset_input_buffer()

        rospy.loginfo(str(self.ser))

        self.arduino = ArduinoSerial();

        rospy.loginfo('Initialized serial')

        arduino_current_avg = 0

        while not rospy.is_shutdown():
            current_time = rospy.get_rostime()
            if((current_time-self.last_command_time).to_sec() > 0.5):
                self.control_mode = 0     # disable motor
                self.new_msg = True
                rospy.loginfo('Motor disabled since no input signal for 0.5s')
            if self.serial_ticks >= self.rate_main/self.serial_rate:
                self.serial_ticks = 0
                self.new_msg = True

            if self.new_msg:
                if self.control_mode == 0:          # disarmed
                    msg = self.sendDisarm()
                elif self.control_mode == 1:        # normal
                    msg = self.sendThrottle()
                elif self.control_mode == 3:        # command
                    msg = self.sendCommand()
                # msg.header.stamp = rospy.get_rostime()
                pub.publish(msg)
                self.arduino.write(msg.data)
                # self.arduino.printstats()
                # rospy.loginfo('Motor speed: %s', msg.data)
                # if(self.arduino.serial.in_waiting > 1)
                arduino_data = self.arduino.read_str() # 0-1023 for 0-2.53V
                # if(msg.data != int(arduino_data)):
                #     print("warning: Target from Arduino=" + arduino_data\
                #     + ", msg.data = " + str(msg.data))  # target
                # print("Arduino data: " + str(arduino_data))
                if(len(arduino_data) > 0 ):  
                    arduino_current_avg = (arduino_current_avg*3 +(1023.0/2530*float(arduino_data[:-2])))/4 # mV avg.
                self.tlm_current = ((arduino_current_avg)/15.2) #- 0.5    # 5000mV/((1<<10)-1)*current_read/15.2(mV/A) =[A] # - offset
                # else 
                # self.tlm_current = -1

                # print("Current: " + str(self.tlm_current) + " A")
                self.new_msg = False

            tlm_msg = self.get_tlm(self.ser)
            if tlm_msg.temperature != 0:
                tlm_msg.header.stamp = rospy.get_rostime()
                tlm_msg.current = self.tlm_current
                tlm_pub.publish(tlm_msg)

            self.serial_ticks += 1
            self.rate.sleep()

    def sendDisarm(self):
        self.throttle = 48
        msg = cmd_msg()
        msg.data = 48
        return msg

    def sendThrottle(self):
        msg = cmd_msg()
        msg.data = self.throttle
        return msg

    def sendCommand(self):
        msg = cmd_msg()
        msg.data = 48
        return msg

    # def update_crc8(self, crc, crc_seed):
    #     crc_u = int.from_bytes(crc, sys.byteorder)
    #     crc_u = (crc_u ^ crc_seed)
    #     for i in [0,8]:
    #         if crc_u & 0x80:
    #             crc_u = 0x7 ^ ( crc_u << 1 )
    #         else:
    #             crc_u = crc_u << 1
    #     return (crc_u);
        
    # def get_crc8(self, Buf, BufLen):
    #     crc = 0
    #     for i in [0, BufLen]:
    #         crc = self.update_crc8(bytes(Buf[i]), crc)
    #     return (crc);

    # ESC Telemetry
        # Byte 0: Temperature 
        # Byte 1: Voltage high byte
        # Byte 2: Voltage low byte
        # Byte 3: Current high byte
        # Byte 4: Current low byte
        # Byte 5: Consumption high byte
        # Byte 6: Consumption low byte
        # Byte 7: Rpm high byte
        # Byte 8: Rpm low byte
        # Byte 9: 8-bit CRC

        # Converting the received values to standard units
        # int8_t Temperature = Temperature in 1 degree C
        # uint16_t Voltage = Volt *100 so 1000 are 10.00V
        # uint16_t Current = Ampere * 100 so 1000 are 10.00A
        # uint16_t Consumption = Consumption in 1mAh
        # uint16_t ERpm = Electrical Rpm /100 so 100 are 10000 Erpm
        # note: to get the real Rpm of the motor you will need to divide the Erpm result by the magnetpole count divided by two.
        # So with a 14magnetpole motor:
        # Rpm = Erpm/7
        # rpm = erpm / (motor poles/2)
    def get_tlm(self, ser):
        # if(ser.in_waiting):
        #     print(ser.in_waiting, ser.out_waiting, ser.get_settings())
        tlm_msg = esc_tlm_msg()
        if(ser.in_waiting >= 10):
            sequence = ser.read(10)
            # sequence = []
            if(len(sequence)>0):
                crc8_hash = crc8.crc8()
                # sequence = ser.read(10)
                tlm =  serial.to_bytes(sequence)
                crc8_hash.update(tlm)
                tlm = struct.unpack('>10B', tlm)
                # print(tlm)

                if(crc8_hash.digest()==b'\0'):
                # if(self.get_crc8(tlm, 10) == 0):
                    tlm_msg.temperature = tlm[0]                    # degrees C
                    tlm_msg.voltage = (tlm[1] << 8 | tlm[2] )/100.0   # V
                    tlm_msg.current = (tlm[3] << 8 | tlm[4] )/100.0   # A
                    tlm_msg.totalcurrent = (tlm[5] << 8 | tlm[6])   # mAh
                    tlm_msg.rpm = (tlm[7] << 8 | tlm[8])            # Erpm
                    #tlm.rpm = tlm.Erpm*200/poles    # rpm
                else:
                    rospy.loginfo('tlm crc error: %i, %s', crc8_hash.hexdigest(), tlm)
        return tlm_msg 


    def set_esc_callback(self, msg):
        self.last_command_time = rospy.get_rostime()
        self.control_mode = 1;  # edit later
        if self.throttle != msg.data:
            self.throttle = msg.data
            self.new_msg = True;
            # rospy.loginfo('new throttle:' + str(self.throttle))


if __name__ == '__main__':
    rospy.init_node("udoo_node", anonymous=True)
    try:
        cn = ESCNode()
    except rospy.ROSInterruptException:
        pass

# TODO Close serial

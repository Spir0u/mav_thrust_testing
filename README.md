# Thrust Test
This code uses the Thrust Test Stand from ASL with the rokubi_mini torque and force sensors.
The configuration is sent over the network to the Udoo board, which sends the data over serial (as hex ascii) to the built-in Arduino. The Arduino sends the throttle commands over the D-Shot protocol to the ESC. The ESC sends back telemetry data over UART to the Udoo board (ESC temperature, voltage, Erpm) periodically.

## Electrical Setup
### Top row:
| Udoo Pin Arduino | Function |
|------------------|----------|
| 30               | ESC GND  |
| 18               | ESC M1   |
| 1                | ESC CURR |
### Bottom Row
| Udoo Pin Embedded | Function |
|-------------------|----------|
| 31                | ESC GND  |
| 33                | ESC TELE |

## Configuration

Source: https://wiki.ros.org/ROS/Tutorials/MultipleMachines

The controller is on the laptop, the listener is the UDOO board.
To get the IP-address `hostname -I` returns IPv4 IPv6

hal is master IP
udoo is listener IP
ssh is just to know where it runs, if you have physical access to both machines

### First Setup/ When something goes wrong

If something in the above sequence didn't work, the cause is likely in your network configuration. See ROS/NetworkSetup and ROS/Troubleshooting for configuration requirements and troubleshooting tips.

One common trap is the missing define of ROS_IP on the machine, where talker.py is running.

check it with: `echo $ROS_IP`

If you dont't define ROS_IP, then rostopic info will show indeed the proper connections of publisher and listener, but rostopic echo will be empty. You will see no TX-traffic on LAN, on machine with talker. First, after defining ROS_IP with proper IP-address ( `export ROS_IP=machine_ip_addr`) you will see trafic on LAN and the listener.py will show received data. 

### Start the master
ssh hal
roscore
(or with node_manager)

### Start the talker

Now we'll start a talker on hal, configuring ROS_MASTER_URI so that we use the master that was just started:

ssh hal
export ROS_MASTER_URI=http://hal:11311
roslaunch thrust_test_controller thrust_test.launch

rqt

rqt_graph

rostopic list

rostopic echo /topic_name

### Start the listener

Next we'll start a listener on udoo, also configuring ROS_MASTER_URI so that the master on hal is used:

ssh udoo
export ROS_MASTER_URI=http://hal:11311
roslaunch thrust_test_receiver thrust_test.launch
<!-- rosrun rospy_tutorials talker.py -->


### Flybook
`
hostname -I
export ROS_IP=
roscore
roslaunch thrust_test_controller thrust_test.launch
`

### Udoo
`
hostname -I
export ROS_IP=
export ROS_MASTER_URI=http://ip:11311
roslaunch thrust_test_receiver thrust_test.launch
`

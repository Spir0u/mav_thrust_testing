## Configuration

The controller is on the laptop, the listener is the UDOO board.
To get the IP-address `hostname -I` returns IPv4 IPv6

hal is master IP
udoo is listener IP
ssh is just to know where it runs, if you have physical access to both machines

# First Setup/ When something goes wrong

If something in the above sequence didn't work, the cause is likely in your network configuration. See ROS/NetworkSetup and ROS/Troubleshooting for configuration requirements and troubleshooting tips.

One common trap is the missing define of ROS_IP on the machine, where talker.py is running.

check it with: `echo $ROS_IP`

If you dont't define ROS_IP, then rostopic info will show indeed the proper connections of publisher and listener, but rostopic echo will be empty. You will see no TX-traffic on LAN, on machine with talker. First, after defining ROS_IP with proper IP-address ( `export ROS_IP=machine_ip_addr`) you will see trafic on LAN and the listener.py will show received data. 

# Start the master
ssh hal
roscore

# Start the listener

Now we'll start a listener on hal, configuring ROS_MASTER_URI so that we use the master that was just started:

ssh hal
export ROS_MASTER_URI=http://hal:11311
roslaunch thrust_test_controller thrust_test.launch

rqt

# Start the talker

Next we'll start a talker on udoo, also configuring ROS_MASTER_URI so that the master on hal is used:

ssh udoo
export ROS_MASTER_URI=http://hal:11311
rosrun rospy_tutorials talker.py

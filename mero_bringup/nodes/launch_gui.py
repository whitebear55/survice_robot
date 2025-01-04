#!/usr/bin/env python3
import rospy, subprocess
import os
import signal

from std_msgs.msg import String

def order_callback(msg):
    global pid
    if msg.data == "navi on":
        command = "roslaunch mero_bringup mero_navigation.launch"
        process = subprocess.Popen(command, shell=True)
        pid = process.pid
    elif msg.data == "navi off":
        os.killpg(os.getpgid(pid), signal.SIGTERM)

if __name__ == '__main__':
    rospy.init_node('launch_node')
    order_sub = rospy.Subscriber("order", String, order_callback, queue_size=1)
    while(1):
        rospy.spin()


#!/usr/bin/env python

import time
import maxSonarTTY
import sys
import signal
import rospy
from std_msgs.msg import Float32


serialPort = "/dev/ttyUSB0"
maxRange = 2000  # change for 5m vs 10m sensor
sleepTime = 0.5
minMM = 10000
maxMM = 0

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class sonar():
    def __init__(self):
        rospy.init_node('sonar', anonymous=True)
        self.distance_publisher = rospy.Publisher('/sonar1_dist',Float32, queue_size=1)
        self.r = rospy.Rate(15)
    def dist_sendor(self,dist):
        data = Float32()
        data.data=dist
        self.distance_publisher.publish(data)

sensor=sonar()
time.sleep(0.5)



while True:
    mm = maxSonarTTY.measure(serialPort)
    if mm >= maxRange:
        print("no target")
        time.sleep(sleepTime)
        continue
    if mm < minMM:
        minMM = mm
    if mm > maxMM:
        maxMM = mm

    print("distance:", mm, "  min:", minMM, "max:", maxMM)
    time.sleep(sleepTime)
    sensor.dist_sendor(mm)
    sensor.r.sleep()
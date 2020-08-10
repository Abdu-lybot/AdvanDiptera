#!/usr/bin/env python

import time
from serial import Serial
import sys
import signal
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range

class sonar():

    def __init__(self):

        rospy.init_node('sonar', anonymous=True)
        topic_name_d = '/sonarTP_D'
        self.distance_publisher_down = rospy.Publisher(topic_name_d,Range, queue_size=5)

        #self.r = rospy.Rate(40)
        self.serialPort0 = "/dev/ttyUSB0"
        self.serialPort1 = "/dev/ttyUSB0"
        self.serialPort2 = "/dev/ttyUSB0"
        self.serialPort3 = "/dev/ttyUSB0"
        self.serialPort4 = "/dev/ttyUSB0"

        self.maxRange = 2000  # change for 5m vs 10m sensor
        self.sleepTime = 0.1
        self.minMM = 0.3
        self.maxMM = 3
        self.mm = float

        r = Range()
        r.header.stamp = rospy.Time.now()
        r.header.frame_id = "/sonarD_link"
        r.radiation_type = 0
        r.field_of_view = 0.8
        self.min_range = self.minMM
        self.max_range = self.maxRange
        r.min_range = self.min_range
        r.max_range = self.max_range
        self._range = r

    def signal_handler(signal, frame):  # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)


    def dist_sendor(self):
        #data = Range()
        valueCount = 0
        ser = Serial(self.serialPort0, 9600, 8, 'N', 1, timeout=100000)
        while not rospy.is_shutdown():
            
            #valueCount += 1
            if ser.inWaiting():
                bytesToRead = ser.inWaiting()
           #     if valueCount < 2: # 1st reading may be partial number; throw it out
            #        continue
            #    valueCount = 0
                testData = ser.read(bytesToRead)
                if not testData.startswith(b'R'):
                    # data received did not start with R
                    continue
                try:
                    sensorData = testData.decode('utf-8').lstrip('R')
                except UnicodeDecodeError:
                    # data received could not be decoded properly
                    continue
                try:
                    mm = int(sensorData)
                except ValueError:
                    # value is not a number
                    continue
            
                self.mm = mm

                if self.mm >= self.maxRange:
                    print("no target")
                    time.sleep(self.sleepTime)
                    continue
                # ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
                self._range.range = self.mm * 0.001
                self._range.header.frame_id = "/sonarD_link"
                self._range.header.stamp = rospy.Time.now()
                self.distance_publisher_down.publish(self._range)
            #ser.close()
        ser.close()

if __name__ == '__main__':
    try:
        sensor = sonar()
        sensor.dist_sendor()

    except rospy.ROSInterruptException: pass
#

#print("distance:",sensor.mm, "  min:", sensor.minMM, "max:", sensor.maxMM)

#sensor.r.sleep()




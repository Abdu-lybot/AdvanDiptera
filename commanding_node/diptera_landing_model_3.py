#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from sensor_msgs.msg import Imu, NavSatFix
from pyquaternion import Quaternion
from geometry_msgs.msg import PoseStamped, Twist
import time
import yaml
from gpio_diptera import Rpi_gpio_comm as Gpio_start
from gpio_clean import Rpi_gpio_comm_off as Gpio_stop

from move_drone import Move_Drone as Basic_movement

class Landing:

    def __init__(self):

        self.yamlpath = '/home/lybot/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as f:
    
           data = yaml.load(f, Loader=yaml.FullLoader)
           for key, value in data.items():
              if key == "land_height":
                 self.land_height = value         # First stop of the drone to land

        # Should be put in a yaml file
        self.threshold = 0.3                      # Threshold to first stop of land
        self.final_land_threshold = 0.2           # When it lands to the ground, minimum distance threshold to finish the landing
        self.landing_step_distance = 0.05         # Cm that the drone will go descending each second


        self.down_sensor_distance = 0             # Initialite to 0

        rospy.init_node("Landing_node")
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback) 
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback) 
        subscribed_topic_d = "/sonarTP_D"
        self.down_sensor = rospy.Subscriber("/sonarTP_D", Range, self.cb_down_sensor)


    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw

        self.received_imu = True

    def q2yaw(self, q):

        if isinstance(q, Quaternion): # Checks if the variable is of the type Quaternion
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z) # Converts into Quaternion
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad 

    def local_pose_callback(self, msg): 

        self.local_pose = msg
        self.local_enu_position = msg


    def cb_down_sensor (self, msg):
        self.down_sensor_distance = msg.range 


    def start(self): # First with Offboard mode, put the drone in a certain alttitude. When the drone is in that position or lower, lands it. 

        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        Basic_movement().moving_to_z(self.land_height) # Probably not necesary here

        while self.local_pose.pose.position.z > (self.land_height + self.threshold) and self.down_sensor_distance > (self.land_height + self.threshold):
            Basic_movement().moving_to_z(self.land_height)
            time.sleep(0.2)

        for self.local_pose.pose.position.z > self.final_land_threshold and self.down_sensor_distance > self.final_land_threshold:
            Basic_movement().moving_down(self.landing_step_distance)
            time.sleep(1)
     
                


if __name__ == '__main__':
    land = Landing()
    land.start
    time.sleep(8) 
        

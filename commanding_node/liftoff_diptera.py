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

class Lift_Off:

    def __init__(self):


        self.yamlpath = '/home/lybot/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as f:
    
           data = yaml.load(f, Loader=yaml.FullLoader)
           for key, value in data.items():
              if key == "takeoff_height":
                 self.takeoff_height = value

        self.threshold = 0.3                      # Can be changed in params.yaml

        rospy.init_node("Lift_off_node")
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback) 
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback) 
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
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


    # Function to create the message PositionTarget
    def construct_target(self, x, y, z, yaw, yaw_rate = 1): 

        target_raw_pose = PositionTarget() 
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 9 

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose

    def constructCommandTOL(self, x, y, z, yaw, min_pitch = 0):
        command = CommandTOL()
        command.min_pitch = min_pitch
        command.latitude = x
        command.longitude = y
        command.altitude = z
        command.yaw = yaw
        return command

    def modechnge_takeoff(self):
        rospy.init_node("land_node")
        if self.flightModeService(costume_mode='AUTO.TAKEOFF'):       # http://wiki.ros.org/mavros/CustomModes
            rospy.loginfo("succesfully changed mode to AUTO.TAKEOFF")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False

    def modechnge(self):
        rospy.init_node("offboard_node")
        if self.flightModeService(costume_mode='OFFBOARD'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False

    def autotakeoff(self):
        if (self.takeoffService(self.constructCommandTOL(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.takeoff_height, self.current_heading))):
            rospy.loginfo("AdvanDiptera is Landed")
            return True 
        else:
            rospy.loginfo("Failed to land AdvanDiptera")
            return False

    def start(self):

        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        self.takeoff_state = self.modechnge_takeoff()

        while self.local_pose.pose.position.z < (self.land_height + self.threshold) and self.cb_down_sensor < (self.land_height + self.threshold):
            self.autotakeoff
            time.sleep(0.2)

        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.takeoff_height,
                                                     self.current_heading)

        self.offboard_state = self.modechnge()
        self.local_target_pub.publish(self.cur_target_pose)
        time.sleep(0.2)
        




        # Here we will have to call the arm service - in our case python script



if __name__ == '__main__':
    takeoff = Lift_Off()
    takeoff.start
    time.sleep(5) 





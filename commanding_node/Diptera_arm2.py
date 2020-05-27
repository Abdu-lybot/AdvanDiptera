#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget
from sensor_msgs.msg import Imu, NavSatFix
from quaternion import Quaternion
import time
import yaml
from gpio_diptera import Rpi_gpio_comm as Gpio_start
from gpio_clean import Rpi_gpio_comm_off as Gpio_stop
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, Twist


class Arming_Modechng():

    def __init__(self):
        self.yamlpath = '/home/ubuntu/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file)
            for key, value in data.items():
                if key == "initial_x_pos":
                    self.init_x = value
                if key == "initial_y_pos":
                    self.init_y = value
                if key == "initial_z_pos":
                    self.init_z = value
                if key == "initial_thrust":
                    self.init_thrust = value
                if key == "accumulating_thrust":
                    self.accumulating_thrust = value
                if key == "Hover_thrust":
                    self.hover_thrust = value
                if key == "Hover_time":
                    self.hover_time = value
                if key == "Hover_sensor_altitude":
                    self.hover_sensor_altitude = value
                if key == "Hover_sensor_altitude_max":
                    self.hover_sensor_altitude_max = value
                if key == "Hover_sensor_altitude_min":
                    self.hover_sensor_altitude_min = value
                if key == "Landing_sensor_altitude_min":
                    self.landing_sensor_altitude_min = value
                if key == "Deaccumulating_thrust":
                    self.Deaccumulating_thrust = value


        self.current_heading = None
        rospy.init_node("Arming_safety_node")
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu,self.imu_callback)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        #self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.attitude_target_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        subscribed_topic_d = "/sonarTP_D"
        self.down_sensor = rospy.Subscriber(subscribed_topic_d, Range, self.cb_down_sensor)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_local_pose)


    def q2yaw(self, q):
        if isinstance(q, Quaternion): # Checks if the variable is of the type Quaternion
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z) # Converts into Quaternion
            rotate_z_rad = q_.yaw_pitch_roll[0]
        return rotate_z_rad

    def cb_local_pose(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg

    def cb_down_sensor(self, msg):
        self.down_sensor_distance = msg.range

    def imu_callback(self, msg):
        global global_imu
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True
        
    def euler2quaternion(self, roll = 0, pitch = 0, yaw = 0):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
        
#----------------------arming services----------------------------
    def arm(self):
        if self.armService(True):
            rospy.loginfo("AdvanDiptera is Armed")
            return True
        else:
            rospy.loginfo("Failed to arm AdvanDiptera")
            return False

    def disarm(self):
        if self.armService(False):
            rospy.loginfo("Advandiptera succesfully disarmed")
            return True
        else:
            rospy.loginfo("Failed to disarm AdvanDiptera")
            return False
        
#----------------------messages constructor----------------------------
    def construct_target(self, x, y, z, yaw, yaw_rate=1):
        target_raw_pose = PositionTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
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
    
    '''    #we should use this in our functions rather then rewriting the message
    def construct_target_attitude(self, body_x = 0, body_y = 0, body_z = 0, thrust = 0.3):
        target_raw_attitude = AttitudeTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        target_raw_attitude.header.stamp = rospy.Time.now()
        #target_raw_attitude.orientation. = self.imu.orientation
        target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE \
                                    + AttitudeTarget.IGNORE_ATTITUDE

        target_raw_attitude.body_rate.x = body_x # ROLL_RATE
        target_raw_attitude.body_rate.y = body_y # PITCH_RATE
        target_raw_attitude.body_rate.z = body_z # YAW_RATE
        target_raw_attitude.thrust = thrust
        return target_raw_attitude
    '''
    
    #----------------------recursive functions----------------------------
    def landing_rec(self, thrust ,beh_type):
        if self.down_sensor_distance <= self.landing_sensor_altitude_min & beh_type == "LANDING":  #we can use also (self.local_pose.pose.position.z <= self.landing_sensor_altitude_min)
            print ("the drone has landed")
            beh_type = "Landed"
            self.disarm()
            return beh_type
        else:
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = self.imu.orientation
            target_raw_attitude.body_rate.x = 0 # ROLL_RATE
            target_raw_attitude.body_rate.y = 0 # PITCH_RATE
            target_raw_attitude.body_rate.z = 0 # YAW_RATE
            beh_type = "LANDING"
            target_raw_attitude.thrust = thrust - self.Deaccumulating_thrust
            self.attitude_target_pub.publish(target_raw_attitude)
            time.sleep(0.02)  #was 0.005 (the recursion depth is depending on velocity)
            return self.landing_rec(thrust, beh_type)   #bublishing a constant parameter "not updating thrust argument"

    def lift_off_rec(self, thrust ,beh_type ,time_flying):
        if time_flying == 0 and beh_type == "HOVER":
            print ("time of hovering has ended")
            beh_type = "LANDING"
            return self.landing_rec(thrust ,beh_type)

        if (thrust == self.hover_thrust) and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min) and time_flying != 0:
            print("the drone is hovering")
            beh_type = 'HOVER'
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = self.imu.orientation
            target_raw_attitude.body_rate.x = 0 # ROLL_RATE
            target_raw_attitude.body_rate.y = 0 # PITCH_RATE
            target_raw_attitude.body_rate.z = 0 # YAW_RATE
            target_raw_attitude.thrust = self.hover_thrust
            self.attitude_target_pub.publish(target_raw_attitude)
            time_flying = self.hover_time - 0.02
            time.sleep(0.02) #was 0.005   (now 50hz ,500loops)
            return self.lift_off_rec(thrust ,beh_type ,time_flying)
        else:
            print("Lifting the drone up slowly")
            target_raw_attitude = AttitudeTarget()
            target_raw_attitude.header.stamp = rospy.Time.now()
            target_raw_attitude.orientation = self.imu.orientation
            target_raw_attitude.body_rate.x = 0 # ROLL_RATE
            target_raw_attitude.body_rate.y = 0 # PITCH_RATE
            target_raw_attitude.body_rate.z = 0 # YAW_RATE
            target_raw_attitude.thrust = thrust + self.accumulating_thrust
            self.attitude_target_pub.publish(target_raw_attitude)
            time.sleep(0.02) # was 0.005 (now 50hz ,500 loops ,5sec)
            return self.lift_off_rec(target_raw_attitude.thrust ,beh_type ,time_flying)



#----------------------change modes----------------------------

    def modechnge(self):
        #rospy.init_node("offboard_node")
        if self.flightModeService(custom_mode='OFFBOARD'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False        
  
    def start(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
        self.cur_target_pose = self.construct_target(self.init_x, self.init_y, self.init_z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose)
        
        self.arm_state = self.arm()
        self.offboard_state = self.modechnge()
        time.sleep(2)
        #self.cur_target_attitude = self.construct_target_attitude()

        #why are we doing this loop
        for i in range(20):
            
            self.arm_state = self.arm()    # arms the drone
            #self.attitude_target_pub.publish(self.cur_target_attitude)
            self.offboard_state = self.modechnge()
            time.sleep(0.1)
            
            
#####################################################################################################            
if __name__ == '__main__':

    try:
        Gpio_start().start()
        time.sleep(3.5)
        print("Waiting for Advandiptera brain")
        arm = Arming_Modechng()
        arm.start()
        arm.lift_off_rec(arm.init_thrust, "Lift_off", arm.hover_time)
    except rospy.ROSInterruptException: pass

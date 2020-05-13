#!/usr/bin/env python


import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from pyquaternion import Quaternion
import time
import yaml
from gpio_diptera import Rpi_gpio_comm as Gpio_start
from gpio_clean import Rpi_gpio_comm_off as Gpio_stop


class Randommovement():


    def __init__(self):


        rospy.init_node("Obstacle_detection_node")

        # Initialize the variables to False

        self.blockingMovementRight = False   
        self.blockingMovementLeft = False 
        self.blockingMovementUp = False 
        self.blockingMovementDown = False
        self.blockingMovementBack = False
        self.blockingMovementForward = False

        self.yamlpath = '/home/lybot/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
            for key, value in data.items():
                if key == "moving_random_distance":
                    self.moving_random_distance = value

        # Previous movement position 
        # 0 right  //  1 left  //  2 back  // 3 forward 
        # Initialize the variable to not previous movement
        # -1 None previous movement
        self.previousMovement = -1 

        self.state = str
        subscribed_topic_state= "/mavros/state"
        self.mavros_sub = rospy.Subscriber(subscribed_topic_state, State, self.cb_mavros_state)
        subscribed_topic_activity = ""     #write here the published topic name "activity"
        self.custom_activity_sub = rospy.Subscriber(subscribed_topic_activity, String, self.cb_activity)
        subscribed_topic_f= "/sonarTP_F"
        self.forward_sensor = rospy.Subscriber(subscribed_topic_f, Range, self.cb_forward_sensor)
        subscribed_topic_r= "/sonarTP_R"
        self.right_sensor = rospy.Subscriber(subscribed_topic_r, Range, self.cb_right_sensor)
        subscribed_topic_l= "/sonarTP_L"
        self.left_sensor = rospy.Subscriber(subscribed_topic_l, Range, self.cb_left_sensor)
        subscribed_topic_b= "/sonarTP_B"
        self.back_sensor = rospy.Subscriber(subscribed_topic_b, Range, self.cb_back_sensor)
        subscribed_topic_d = "/sonarTP_D"
        self.down_sensor = rospy.Subscriber(subscribed_topic_d, Range, self.cb_down_sensor)

        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)

        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)



    def start(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        print("RANDOM MOVE!")
            
        print('IS DIRECTION BLOCKED?')
        print('Blocking Right: ' + str(self.blockingMovementRight))
        print('Blocking Left: ' + str(self.blockingMovementLeft))
        print('Blocking Back: ' + str(self.blockingMovementBack))
        print('Blocking Forward: ' + str(self.blockingMovementForward))
        print('DIRECTION CHOSEN:')


        if self.previousMovement == -1:
            exitloop = False

        elif self.previousMovement == 0:
            if self.blockingMovementRight == False: 
                self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                             self.local_pose.pose.position.y - self.moving_random_distance,
                                                             self.local_pose.pose.position.z,
                                                             self.current_heading)
                print("Random Movement ---> Still Right")
                rospy.loginfo("Random Movement: Still Right!")
                exitloop = True

        elif self.previousMovement == 1:
            if self.blockingMovementLeft == False: 
                self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                             self.local_pose.pose.position.y + self.moving_random_distance,
                                                             self.local_pose.pose.position.z,
                                                             self.current_heading)
                print("Random Movement ---> Still Left")
                rospy.loginfo("Random Movement: Still Left!")
                exitloop = True

        elif self.previousMovement == 2:
            if self.blockingMovementBack == False: 
                self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x - self.moving_random_distance,
                                                             self.local_pose.pose.position.y,
                                                             self.local_pose.pose.position.z,
                                                             self.current_heading)
                print("Random Movement ---> Still Back")
                rospy.loginfo("Random Movement: Still Back!")
                exitloop = True

        elif self.previousMovement == 3:
            if self.blockingMovementForward == False: 
                self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x + self.moving_random_distance,
                                                             self.local_pose.pose.position.y,
                                                             self.local_pose.pose.position.z,
                                                             self.current_heading)
                print("Random Movement ---> Still Forward")
                rospy.loginfo("Random Movement: Still Forward!")
                exitloop = True


        # If it is blocked or first random movement, generate random movement
        while exitloop == False: 

            # generate random integer values
            value = randint(0, 3) # Random int value between 0 and 3

                if value == 0:
                    if self.blockingMovementRight == False:
                        exitloop = True
                        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                                     self.local_pose.pose.position.y - self.moving_random_distance,
                                                                     self.local_pose.pose.position.z,
                                                                     self.current_heading)
                        print("Random Movement ---> Right")
                        rospy.loginfo("Random Movement: Right!")
                        self.previousMovement = 0

                elif value == 1:
                    if self.blockingMovementLeft == False:
                        exitloop = True
                        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                                     self.local_pose.pose.position.y + self.moving_random_distance,
                                                                     self.local_pose.pose.position.z,
                                                                     self.current_heading)
                        print("Random Movement ---> Left")
                        rospy.loginfo("Random Movement: Left!")
                        self.previousMovement = 1

                elif value == 2:
                    if self.blockingMovementBack == False:
                        exitloop = True
                        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x - self.moving_random_distance,
                                                                     self.local_pose.pose.position.y,
                                                                     self.local_pose.pose.position.z,
                                                                     self.current_heading)
                        print("Random Movement ---> Back")
                        rospy.loginfo("Random Movement: Back!")
                        self.previousMovement = 2

                elif value == 3:
                    if self.blockingMovementForward == False:
                        exitloop = True
                        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x + self.moving_random_distance,
                                                                     self.local_pose.pose.position.y,
                                                                     self.local_pose.pose.position.z,
                                                                     self.current_heading)
                        print("Random Movement ---> Forward")
                        rospy.loginfo("Random Movement: Forward!")
                        self.previousMovement = 3


        time.sleep(4) # Rate 



    def cb_mavros_state(self, msg):
        self.mavros_state = msg.mode

    def cb_activity(self, msg):
        print("Received Custom Activity:", msg.data)
        if msg.data == "HOVER":
            print("HOVERING!")
            rospy.loginfo("Hovering!")
            self.state = "HOVER"

    def cb_forward_sensor(self, sense):
        if (self.state == "HOVER"):
            for sense.range not in range(0,1.5):
                print("no obstacle close to drone")
                self.blockingMovementForward = False
            else:
                #move opposite direction with the difference distance
                self.blockingMovementForward = True

    def cb_right_sensor(self,sense):
        if (self.state == "HOVER"):
            for sense.range not in range(0,1.5):
                print("no obstacle close to drone")
                self.blockingMovementRight = False
            else:
                #move opposite direction with the difference distance
                self.blockingMovementRight = True

    def cb_left_sensor(self,sense):
        if (self.state == "HOVER"):
            for sense.range not in range(0,1.5):
                print("no obstacle close to drone")
                self.blockingMovementLeft = False
            else:
                #move opposite direction with the difference distance
                self.blockingMovementLeft = True 

    def cb_back_sensor(self,sense):
        if (self.state == "HOVER"):
            for sense.range not in range(0,1.5):
                print("no obstacle close to drone")
                self.blockingMovementBack = False
            else:
                #move opposite direction with the difference distance
                self.blockingMovementBack = True


    def construct_target(self, x, y, z, yaw, yaw_rate = 1): 
        target_raw_pose = PositionTarget() # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
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

    def local_pose_callback(self, msg): 
        self.local_pose = msg
        self.local_enu_position = msg


    def imu_callback(self, msg):

        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw

        self.received_imu = True

if __name__ == '__main__':

    sensing = Randommovement()
    sensing.start()





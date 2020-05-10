import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
import time


class Arming_Modechng():

    def __init__(self):
        rospy.init_node("Arming_safety_node")
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

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

    def modechnge(self):
        rospy.init_node("offboard_node")
        if self.flightModeService(costume_mode='OFFBOARD'):
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

        self.cur_target_pose = self.construct_target(0, 0, 0, self.current_heading) # Initialize the drone to this current position

        #print ("self.cur_target_pose:", self.cur_target_pose, type(self.cur_target_pose))

        for i in range(10):
            self.local_target_pub.publish(self.cur_target_pose) # Publish the drone position we initialite during the first 2 seconds
            #self.arm_state = self.arm() # Arms the drone (not necessary here)
            self.offboard_state = self.modechnge() # Calls the function offboard the will select the mode Offboard
            time.sleep(0.2)
            while self.offboard_state and (
                    rospy.is_shutdown() is False):  # While offboard state is true and we don't shutdown it, do the loop

                self.local_target_pub.publish(
                    self.cur_target_pose)  # Publish to the mavros local_targe_pub our desired new position

                if (self.state is "LAND") and (
                        self.local_pose.pose.position.z < self.threshold_ground_minor):  # If we land and we are under 0.15 in the z position...

                    if (self.disarm()):  # ... we disarm the drone

                        self.state = "DISARMED"

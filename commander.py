import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
import time
import math


class Commander:
    def __init__(self):
        rospy.init_node("commander_node") # We initialize the node with the name: commander_node
        rate = rospy.Rate(20) # Rate of 20 Hz
        self.position_target_pub = rospy.Publisher('gi/set_pose/position', PoseStamped, queue_size=10) # Custom publisher of the position target
        self.yaw_target_pub = rospy.Publisher('gi/set_pose/orientation', Float32, queue_size=10) # Custom publisher of the yaw target
        self.custom_activity_pub = rospy.Publisher('gi/set_activity/type', String, queue_size=10) # Custom publisher of some Strings which will evolve in an specific function
        self.custom_takeoff = rospy.Publisher('gi/set_activity/takeoff', Float32, queue_size=10) # Custom publisher of a takeoff + hight

    def move(self, x, y, z, BODY_OFFSET_ENU=True):  # Function called to publish the new position 
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU)) # It also calls the set_pose function


    def turn(self, yaw_degree): # Function called to publish the degrees what we have to yaw
        self.yaw_target_pub.publish(yaw_degree) # It publishes it 

    
    # land at current position
    def land(self): # Function called to land the drone
        self.custom_activity_pub.publish(String("LAND")) # It publishes the string LAND


    # hover at current position
    def hover(self): # Function called to hover the drone
        self.custom_activity_pub.publish(String("HOVER")) # It publishes the string HOVER


    # takeoff
    def takeoff(self): # Function called to takeoff the drone but still in the height position desired in the other python script
        self.custom_activity_pub.publish(String("TAKEOFF")) # It publishes the string TAKEOFF


    # takeoff custom
    def takeoff_custom(self, height): # Function called to take off the drone to an specific Z 
        self.custom_takeoff.publish(height)  # It publishes the height

    # return to home position with defined height
    def return_home(self, height): 
        self.position_target_pub.publish(self.set_pose(0, 0, height, False)) # It also calls the set_pose function


    def set_pose(self, x=0, y=0, z=2, BODY_FLU = True): # Function to set the new position
        pose = PoseStamped() # Creates the message PoseStamped which has header + pose
        pose.header.stamp = rospy.Time.now() # Introduce to the stamp parameter of the header the time 

        # ROS uses ENU internally, so we will stick to this convention
        if BODY_FLU: # 
            pose.header.frame_id = 'base_link' # Base of the drone 

        else:
            pose.header.frame_id = 'map' # or the map
        # Introduce to position x,y,z the values we wanted 
        pose.pose.position.x = x 
        pose.pose.position.y = y
        pose.pose.position.z = z
        # We don't touch orientation
        return pose # Return the variable pose


if __name__ == "__main__": # From here to the end we call all the functions in our order desired
    
    con = Commander()
    time.sleep(2)
    print("Take off in progress...")
    con.takeoff_custom(1)
    time.sleep(2)


    print("Moving one meter in X direction in 5 seconds")
    for x in range(5,0,-1):
        print("Moving in... " + str(x))
        time.sleep(1)
    
    print("Moving in progress...")
    con.move(1,0,0)
    time.sleep(2)

    print("Landing in 1 minute")
    time.sleep(5)

    for x in range(10,0,-1):
        print("Landing in... " + str(x))
        time.sleep(1)
    
    print("Landing in progress...") 
    con.land()
    time.sleep(10)



    print("Going home")
    con.takeoff()
    time.sleep(4)
    con.move(-1,0,0)
    time.sleep(4)
    con.land()


import rospy
from pyquaternion import Quaternion
from mavros_msgs.msg import GlobalPositionTarget, State, AttitudeTarget
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, NavSatFix
import time
import yaml


class Move_Drone():

    def __init__(self):
        rospy.init_node("Move_drone_node")

        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.cb_imu)
        self.attitude_target_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)


    def construct_target(self, body_x = 0, body_y=0, body_z = 0, thrust = 0.2):
        target_raw_attitude = AttitudeTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        target_raw_attitude.header.stamp = rospy.Time.now()
        target_raw_attitude.orientation. = self.imu.orientation
        target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE \
                                    + AttitudeTarget.IGNORE_THRUST + AttitudeTarget.IGNORE_ATTITUDE

        target_raw_attitude.body_rate.x = body_x # ROLL_RATE
        target_raw_attitude.body_rate.y = body_y # PITCH_RATE
        target_raw_attitude.body_rate.z = body_z # YAW_RATE
        target_raw_attitude.thrust = thrust
        return target_raw_attitude

    def cb_imu(self, msg):
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True

    def q2yaw(self, q):
        if isinstance(q, Quaternion): # Checks if the variable is of the type Quaternion
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            self.q_ = Quaternion(q.w, q.x, q.y, q.z) # Converts into Quaternion
            rotate_z_rad =self.q_.yaw_pitch_roll[0]
        return rotate_z_rad

    def waiting_initialization(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
    
    # Moves to a determinate location
    def start(self, x, y, z, yaw, yaw_rate = 1):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(x, y, z, yaw, yaw_rate)
        self.local_target_pub.publish(self.cur_target_pose) 

    # Moves a determinate distance
    def move_in_x(self, distance):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x + distance, self.local_pose.pose.position.y, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose) 

    def move_in_y(self, distance):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y + distance, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose) 

    def move_in_z(self, distance):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z + distance, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose) 
    
    # Moves in a determined direction        
    def moving_forward(self, distance):
        self.move_in_x(distance)

    def moving_back(self, distance):
        self.move_in_x(distance)

    def moving_left(self, distance):
        self.move_in_y(-distance)

    def moving_right(self, distance):
        self.move_in_y(-distance)

    def moving_up(self, distance):
        self.move_in_z(distance)

    def moving_down(self, distance):
        self.move_in_z(-distance)

    # Moves to a determinate location
    def move_to_x(self, x_location):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(x_location, self.local_pose.pose.position.y, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose) 

    def move_to_y(self, y_location):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, y_location, self.local_pose.pose.position.z, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose) 

    def move_to_z(self, z_location):
        self.waiting_initialization()
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, z_location, self.current_heading)
        self.local_target_pub.publish(self.cur_target_pose) 

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from geometry_msgs.msg import PoseStamped
import time
import yaml
from gpio_diptera import Rpi_gpio_comm as Gpio_start
from gpio_clean import Rpi_gpio_comm_off as Gpio_stop


class Disarming_Modechng():

    def __init__(self):
        self.yamlpath = '/home/lybot/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
            for key, value in data.items():
                if key == "threshold_ground_minor":
                    self.threshold_ground_minor = value

        self.down_sensor_distance = 0

        rospy.init_node("Disarming_safety_node")
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu,self.imu_callback) 
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        subscribed_topic_d = "/sonarTP_D"
        self.down_sensor = rospy.Subscriber("/sonarTP_D", Range, self.cb_down_sensor)

    def q2yaw(self, q):
        if isinstance(q, Quaternion): # Checks if the variable is of the type Quaternion
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z) # Converts into Quaternion
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad


    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True

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



    def local_pose_callback(self, msg): 

        self.local_pose = msg
        self.local_enu_position = msg

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

    def cb_down_sensor (self, msg):
        self.down_sensor_distance = msg.range


    def start(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.local_pose.pose.position.z is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)


        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, 0, self.current_heading)

        while self.down_sensor_distance > self.threshold_ground_minor and self.local_pose.pose.position.z > self.threshold_ground_minor: # If we land and we are under 0.15 in the z position...
            self.local_target_pub.publish(self.cur_target_pose) # Publish the drone position we initialite during the first 2 seconds
            self.offboard_state = self.modechnge() # Calls the function offboard the will select the mode Offboard
            time.sleep(0.2) # Rate 

        self.disarm() # ... we disarm the drone

            


if __name__ == '__main__':
    try:
        disarm = Disarming_Modechng()
        disarm.start()
        Gpio_stop.start()
        time.sleep(3.5)

    except rospy.ROSInterruptException: pass






import rospy
from std_msgs.msg import Float32, Float64, String
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from sensor_msgs.msg import Range


class UltrasonicSensing():


    def __init__(self):
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
            for sense.range in range(0,1):
                print("no obstacle close to drone")
            else:
                #move opposite direction with the difference distance

    def cb_right_sensor(self,sense):
        if (self.state == "HOVER"):
            for sense.range in range(0,1):
                print("no obstacle close to drone")
            else:
                #move opposite direction with the difference distance

    def cb_left_sensor(self,sense):
        if (self.state == "HOVER"):
            for sense.range in range(0,1):
                print("no obstacle close to drone")
            else:
                #move opposite direction with the difference distance

    def cb_back_sensor(self,sense):
        if (self.state == "HOVER"):
            for sense.range in range(0,1):
                print("no obstacle close to drone")
            else:
                #move opposite direction with the difference distance

    def cb_down_sensor(self,sense):
        if (self.state == "HOVER"):
            for sense.range in range(0,1):
                print("no obstacle close to drone")
            else:
                #move opposite direction with the difference distance
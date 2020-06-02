import rospy
from std_msgs.msg import String


def navigation_node(sensor):
    topic = "navigation"
    pub = rospy.Publisher(topic, String, queue_size=10)
    rospy.init_node('navigator_publisher', anonymous=True)
    # 20hz
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        data_from_sensor = sensor.read_from_sensor()
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(data_from_sensor)
        pub.publish(data_from_sensor)
        rate.sleep()

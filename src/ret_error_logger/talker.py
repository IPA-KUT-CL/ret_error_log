import imp
import rospy
from std_msgs.msg import String
from ret_error_logger.segfault import endless_recur

def talker():
    pub = rospy.Publisher("chatter", String, queue_size=10)
    rospy.init_node("talker", anonymous=False)
    rate = rospy.Rate(1)
    count = 0
    while not rospy.is_shutdown():
        str_msg = "Hello" + str(count)
        if count == 5:
            rospy.loginfo('a info')
            rospy.logerr('calling endless recursion')
            str_msg = "Hello" + endless_recur(10)
        count += 1
        rospy.loginfo(str_msg)
        pub.publish(str_msg)
        rate.sleep()
import rospy
from scipy.sparse.data import _data_matrix
from std_msgs.msg import String
import sys

NODE_NAME = "test_node"
FREQUENCY = 1
pub = None


def main_loop():
    rate = rospy.Rate(FREQUENCY)
    while not rospy.is_shutdown():
        msg = String()
        msg.data = str(rospy.get_published_topics())
        print(rospy.get_published_topics())
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    rospy.loginfo(NODE_NAME + " started")
    pub = rospy.Publisher(f"/{NODE_NAME}/{'test'}", String, queue_size=1)
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

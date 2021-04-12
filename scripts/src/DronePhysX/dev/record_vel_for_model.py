import re
import rospy
from time import time, sleep

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int64, Bool
from geometry_msgs.msg import TwistStamped, PoseStamped, QuaternionStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped

from tf_conversions.posemath import transformations


def get_num_drones():
    drones_found = set()
    p = re.compile(r"/mavros(.*)/local_position/pose")
    for topic in rospy.get_published_topics():
        for element in p.findall(topic[0]):
            drones_found.add(element)
    return len(drones_found)


instances_num = get_num_drones()
actors = {}
data = {}
drone_to_record = 1
file = None
ready = False
start_time = 0.0


def setpoint_topic_cb(msg, callback_args):
    n, suff = callback_args
    data[n][suff] = msg


def vel_topic_cb(msg, callback_args):
    n, suff = callback_args
    data[n][suff] = msg
    if ready and n == drone_to_record:
        control = data[drone_to_record]['setpoint_raw/local']
        file.write(f"{time() - start_time};{msg.twist.linear.x};{msg.twist.linear.y};{msg.twist.linear.z};"
                   f"{control.velocity.x};{control.velocity.y};{control.velocity.z}\n")


def subscribe_on_topics():
    for n in range(1, instances_num + 1):
        data[n] = {}
        suff = "setpoint_raw/local"
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, PositionTarget, setpoint_topic_cb, callback_args=(n, suff))
    for n in range(1, instances_num + 1):
        data[n] = {}
        suff = "local_position/velocity_local"
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, TwistStamped, vel_topic_cb, callback_args=(n, suff))


if __name__ == '__main__':
    try:
        file = open("data/" + str(int(time())) + ".csv", "a+")
        file.write("time;velocity.x;velocity.y;velocity.z;control_vel.x;control_vel.y;control_vel.z\n")
        rospy.init_node("record_vel_for_model")
        subscribe_on_topics()
        sleep(1)
        start_time = time()
        ready = True
        rospy.spin()
    except rospy.ROSInterruptException:
        file.close()

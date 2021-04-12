#!/usr/bin/env python3
# coding=utf8

import rospy
import time
import sys
import math

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int64, Bool
from geometry_msgs.msg import TwistStamped, PoseStamped, QuaternionStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

from sensor_msgs.msg import Joy

control = QuaternionStamped()
mode = String()
global drones
drones = Int64()
drones.data = 1
takeoff = Bool()
global takeoff_done
takeoff_done = False
global lb_done
lb_done = False
global rb_done
rb_done = False
drones.data = 1


def joy_cb(data):
    global takeoff_done
    global drones_counter
    global lb_done
    global rb_done
    control.quaternion.x = -data.axes[0]
    control.quaternion.y = data.axes[1]
    control.quaternion.z = -data.axes[2]
    control.quaternion.w = data.axes[3]
    control.header.stamp = rospy.get_rostime()
    buttons = data.buttons[1] * 2 + data.buttons[2]
    if data.buttons[5] == 0:
        if buttons == 1:
            mode.data = "local vel"
        if buttons == 0:
            mode.data = "local ..."
        if buttons == 2:
            mode.data = "local pos"
    else:
        if buttons == 1:
            mode.data = "global vel"
        if buttons == 0:
            mode.data = "global ..."
        if buttons == 2:
            mode.data = "global pos"
    if data.buttons[0] == 0 and not takeoff_done:
        takeoff.data = True
        takeoff_done = True
    elif data.buttons[0] == 1 and takeoff_done:
        takeoff.data = False
        takeoff_done = False
    else:
        takeoff.data = False
    if data.buttons[6] == 1 and not lb_done:
        if drones.data > 1:
            drones.data -= 1
        lb_done = True
    elif data.buttons[6] == 0 and lb_done:
        lb_done = False
    if data.buttons[7] == 1 and not rb_done:
        drones.data += 1
        rb_done = True
    elif data.buttons[7] == 0 and rb_done:
        rb_done = False


NODE_NAME = "flysky_fs_6s_remap"
FREQUENCY = 100
TIMEOUT = 1.0  # В секундах

rospy.Subscriber("/joy", Joy, joy_cb)

if __name__ == '__main__':
    rospy.init_node(NODE_NAME)
    rospy.loginfo(NODE_NAME + " started")
    control_pub = rospy.Publisher("/joy/control", QuaternionStamped, queue_size=1)
    mode_pub = rospy.Publisher("/joy/control/mode", String, queue_size=1)
    drones_pub = rospy.Publisher("/joy/control/drones", Int64, queue_size=1)
    takeoff_pub = rospy.Publisher("/joy/control/takeoff", Bool, queue_size=1)
    try:
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            if rospy.get_rostime().to_time() - control.header.stamp.to_time() > TIMEOUT:
                control.quaternion.x = 0
                control.quaternion.y = 0
                control.quaternion.z = 0
                control.quaternion.w = 0
            control_pub.publish(control)
            mode_pub.publish(mode)
            takeoff_pub.publish(takeoff)
            drones_pub.publish(drones)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

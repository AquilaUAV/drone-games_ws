#!/usr/bin/env python3
# coding=utf8

from random import random

import rospy
from math import *
from time import sleep, time
import re

from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import String

freq = 30
node_name = "multirotor_easy_controller"
data = {}


def get_num_drones():
    drones_found = set()
    p = re.compile(r"/mavros(.*)/local_position/pose")
    for topic in rospy.get_published_topics():
        for element in p.findall(topic[0]):
            drones_found.add(element)
    return len(drones_found)


instances_num = get_num_drones()


def subscribe_on_mavros_topics(suff, data_class):
    for n in range(1, instances_num + 1):
        data[n] = {}
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, data_class, topic_cb, callback_args=(n, suff))


def subscribe_on_control_topics():
    rospy.Subscriber("/path_generator/_central", String, control_cb, callback_args=("/path_generator/_central",))
    rospy.Subscriber("/path_generator/_walls", String, control_cb, callback_args=("/path_generator/_walls",))


def control_cb(msg, callback_args):
    suff, = callback_args
    data[suff] = msg


def topic_cb(msg, callback_args):
    n, suff = callback_args
    data[n][suff] = msg


def service_proxy(n, path, arg_type, *args, **kwds):
    service = rospy.ServiceProxy(f"/mavros{n}/{path}", arg_type)
    ret = service(*args, **kwds)
    # rospy.loginfo(f"{n}: {path} {args}, {kwds} => {ret}")


def subscribe_on_topics():
    subscribe_on_mavros_topics("local_position/pose", PoseStamped)
    subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)
    subscribe_on_mavros_topics("state", State)
    subscribe_on_mavros_topics("extended_state", ExtendedState)
    subscribe_on_control_topics()


def on_shutdown_cb():
    rospy.logfatal("shutdown")


def arming(n, to_arm):
    d = data[n].get("state")
    if d is not None and d.armed != to_arm:
        service_proxy(n, "cmd/arming", CommandBool, to_arm)


def set_mode(n, new_mode):
    d = data[n].get("state")
    if d is not None and d.mode != new_mode:
        service_proxy(n, "set_mode", SetMode, custom_mode=new_mode)


def mc_takeoff(pt, n, dt):
    if dt < 0.4:
        set_vel(pt, 0, 0, 0.3)
        if dt > 0.2:
            arming(n, True)


def set_pos(pt, x, y, z):
    global drone_yaw
    pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ
    pt.position.x = x
    pt.position.y = y
    pt.position.z = z


def set_vel(pt, x, y, z):
    global drone_yaw
    pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ
    pt.velocity.x = x
    pt.velocity.y = y
    pt.velocity.z = z


# TODO: Свой код управления пишите ниже:
walls = {}
central = {}


def control(pt, n, dt):
    mc_takeoff(pt, n, dt)
    walls_new = data.get("/path_generator/_walls")
    central_new = data.get("/path_generator/_central")
    copter_pose = data[n].get("local_position/pose")
    if walls_new is None or central_new is None or copter_pose is None:
        return
    walls_new = walls_new.data.split(' ')
    walls[walls_new[1]] = []
    for i in range(len(walls_new[2:]) // 4):
        walls[walls_new[1]].append([float(num) for num in walls_new[(2 + i * 4):(2 + i * 4) + 4]])
    central_new = central_new.data.split(' ')
    central[central_new[1]] = []
    for i in range(len(central_new[2:]) // 3):
        central[central_new[1]].append([float(num) for num in central_new[(2 + i * 3):(2 + i * 3) + 3]])
    copter_pose = copter_pose.pose.position
    copter_pose = [copter_pose.x, copter_pose.y, copter_pose.z]

    # TODO: Свой код управления пишите тут:

    # print(_walls)
    # print(_central)
    # print(copter_pose)
    set_pos(pt, sin(dt + 2 * pi * n / instances_num), cos(dt + 2 * pi * n / instances_num), n)


# TODO: Свой код управления пишите выше:

def offboard_loop():
    pub_pt = {}

    for n in range(1, instances_num + 1):
        pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)

    pt = PositionTarget()
    pt.coordinate_frame = pt.FRAME_LOCAL_NED

    global t0
    t0 = time()

    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        dt = time() - t0
        for n in range(1, instances_num + 1):
            set_mode(n, "OFFBOARD")
            control(pt, n, dt)
            pub_pt[n].publish(pt)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node(node_name)
    rospy.loginfo(f"{node_name} started with {instances_num} drones")

    subscribe_on_topics()
    rospy.on_shutdown(on_shutdown_cb)

    try:
        offboard_loop()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

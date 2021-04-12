#!/usr/bin/env python3
# coding=utf8

from random import random

import rospy
import time
import sys
import math
import re

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int64, Bool
from geometry_msgs.msg import TwistStamped, PoseStamped, QuaternionStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

global t0
global drone_yaw
global YAW_RATE
global target_x
global target_y
global target_z
global drones_num
global drones_mode
drones_num = 0
drones_mode = ""

target_x = 0.0
target_y = 0.0
target_z = 0.0

drone_yaw = 0
yaw_rate = 0

MAX_LINEAR_PLANE = 12
MAX_LINEAR_VERTICAL = 1
MAX_ANG_VEL = -4
POINT_VEL_K = 0.2

freq = 100
node_name = "offboard_node"
data = {}
lz = {}

R = 2


def get_num_drones():
    drones_found = set()
    p = re.compile(r"/mavros(.*)/local_position/pose")
    for topic in rospy.get_published_topics():
        for element in p.findall(topic[0]):
            drones_found.add(element)
    return len(drones_found)


instances_num = get_num_drones()


def control_pos_formation_vector(n):
    matrix = {
        1: [-2 * R, -2 * R, 0],
        2: [-2 * R, 2 * R, 0],
        3: [2 * R, -2 * R, 0],
        4: [2 * R, 2 * R, 0],
        5: [-R, -R, R],
        6: [-R, R, R],
        7: [R, -R, R],
        8: [R, R, R],
        9: [-R, -R, 2 * R],
        10: [-R, R, 2 * R],
        11: [R, -R, 2 * R],
        12: [R, R, 2 * R],
        13: [-2 * R, -2 * R, 3 * R],
        14: [-2 * R, 2 * R, 3 * R],
        15: [2 * R, -R, 3 * R],
        16: [2 * R, 2 * R, 3 * R],
    }
    if n not in matrix.keys():
        return [0, 0, 0]
    return matrix.get(n)


def subscribe_on_mavros_topics(suff, data_class):
    for n in range(1, instances_num + 1):
        data[n] = {}
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, data_class, topic_cb, callback_args=(n, suff))


def subscribe_on_control_topics():
    rospy.Subscriber("/joy/control", QuaternionStamped, control_cb, callback_args=("/joy/control",))
    rospy.Subscriber("/joy/control/mode", String, control_cb, callback_args=("/joy/control/mode",))
    rospy.Subscriber("/joy/control/drones", Int64, control_cb, callback_args=("/joy/control/drones",))
    rospy.Subscriber("/joy/control/takeoff", Bool, control_cb, callback_args=("/joy/control/takeoff",))


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
    pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW_RATE
    pt.position.x = x
    pt.position.y = y
    pt.position.z = z
    pt.yaw = drone_yaw


def set_vel(pt, x, y, z):
    global drone_yaw
    pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW_RATE
    pt.velocity.x = x
    pt.velocity.y = y
    pt.velocity.z = z
    pt.yaw = drone_yaw


def control(pt, n, dt):
    global t0
    global yaw_rate
    global drone_yaw
    global target_x
    global target_y
    global target_z
    global drones_num
    if data.get('/joy/control') is None \
            or data.get('/joy/control/mode') is None \
            or data.get('/joy/control/drones') is None \
            or data.get('/joy/control/takeoff') is None:
        return
    if data.get('/joy/control/takeoff').data:
        rospy.logwarn(f"TAKEOFF")
        t0 = time.time()
        data.get('/joy/control/takeoff').data = False
    set_vel(pt, 0, 0, 0)
    mc_takeoff(pt, n, dt)
    if n <= drones_num:
        if data.get('/joy/control/mode').data == "global vel":
            yaw_rate = data.get('/joy/control').quaternion.w * MAX_ANG_VEL
            set_vel(pt, data.get('/joy/control').quaternion.x * MAX_LINEAR_PLANE,
                    data.get('/joy/control').quaternion.y * MAX_LINEAR_PLANE,
                    data.get('/joy/control').quaternion.z * MAX_LINEAR_VERTICAL)
        if data.get('/joy/control/mode').data == "local vel":
            yaw_rate = data.get('/joy/control').quaternion.w * MAX_ANG_VEL
            teta = drone_yaw + yaw_rate / freq - math.pi / 2
            global_x = data.get('/joy/control').quaternion.x * MAX_LINEAR_PLANE
            global_y = data.get('/joy/control').quaternion.y * MAX_LINEAR_PLANE
            local_x = global_x * math.cos(teta) - global_y * math.sin(teta)
            local_y = global_x * math.sin(teta) + global_y * math.cos(teta)
            set_vel(pt, local_x, local_y, data.get('/joy/control').quaternion.z * MAX_LINEAR_VERTICAL)
        if data.get('/joy/control/mode').data == "global pos":
            teta = drone_yaw + yaw_rate / freq - math.pi / 2
            yaw_rate = data.get('/joy/control').quaternion.w * MAX_ANG_VEL
            dx, dy, dz = control_pos_formation_vector(n)
            local_dx = dx * math.cos(teta) - dy * math.sin(teta)
            local_dy = dx * math.sin(teta) + dy * math.cos(teta)
            target_x += data.get('/joy/control').quaternion.x * MAX_LINEAR_PLANE / freq * POINT_VEL_K
            target_y += data.get('/joy/control').quaternion.y * MAX_LINEAR_PLANE / freq * POINT_VEL_K
            target_z += data.get('/joy/control').quaternion.z * MAX_LINEAR_VERTICAL / freq * POINT_VEL_K
            set_pos(pt, target_x + local_dx, target_y + local_dy, target_z + dz)
        if data.get('/joy/control/mode').data == "local pos":
            teta = drone_yaw + yaw_rate / freq - math.pi / 2
            yaw_rate = data.get('/joy/control').quaternion.w * MAX_ANG_VEL * POINT_VEL_K
            dx, dy, dz = control_pos_formation_vector(n)
            local_dx = dx * math.cos(teta) - dy * math.sin(teta)
            local_dy = dx * math.sin(teta) + dy * math.cos(teta)
            global_x = data.get('/joy/control').quaternion.x * MAX_LINEAR_PLANE
            global_y = data.get('/joy/control').quaternion.y * MAX_LINEAR_PLANE
            target_x += (global_x * math.cos(teta) - global_y * math.sin(teta)) / freq * POINT_VEL_K
            target_y += (global_x * math.sin(teta) + global_y * math.cos(teta)) / freq * POINT_VEL_K
            target_z += data.get('/joy/control').quaternion.z * MAX_LINEAR_VERTICAL / freq * POINT_VEL_K
            set_pos(pt, target_x + local_dx, target_y + local_dy, target_z + dz)


def offboard_loop():
    global drone_yaw
    global yaw_rate
    global drones_num
    global drones_mode

    pub_pt = {}

    for n in range(1, instances_num + 1):
        pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)

    pt = PositionTarget()

    pt.coordinate_frame = pt.FRAME_LOCAL_NED

    global t0
    t0 = time.time()

    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        if data.get('/joy/control/drones') is None:
            rate.sleep()
            continue
        if data.get('/joy/control/drones').data > instances_num:
            rospy.logerr(f"Can't control: {data.get('/joy/control/drones').data} drones, limit is {instances_num}")
            data.get('/joy/control/drones').data = instances_num
        if data.get('/joy/control/drones').data != drones_num:
            drones_num = data.get('/joy/control/drones').data
            rospy.logwarn(f"Drones controlled: {drones_num}")
        if data.get('/joy/control/mode').data != drones_mode:
            drones_mode = data.get('/joy/control/mode').data
            rospy.logwarn(f"Control mode: {drones_mode}")

        drone_yaw += yaw_rate * 1 / freq
        drone_yaw = drone_yaw % (2 * math.pi)
        dt = time.time() - t0

        for n in range(1, drones_num + 1):
            set_mode(n, "OFFBOARD")
            control(pt, n, dt)
            pub_pt[n].publish(pt)

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node(node_name)
    rospy.loginfo(node_name + " started")

    subscribe_on_topics()

    rospy.on_shutdown(on_shutdown_cb)

    try:
        offboard_loop()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

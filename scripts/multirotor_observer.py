#!/usr/bin/env python3
# coding=utf8

import re
import rospy
from time import time, sleep
from math import *
from pyphysx import *
from pyphysx_utils.rate import Rate
from pyphysx_render.pyrender import PyPhysxViewer
from pyphysx_render.meshcat_render import MeshcatViewer
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist, Transform
from mavros_msgs.msg import PositionTarget
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

scene = Scene(scene_flags=[ENABLE_GPU_DYNAMICS, ], )

node_name = "multirotor_observer"


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
for n in range(1, instances_num + 1):
    data[n] = {}
global control_mode_velocities
control_mode_velocities = False

publish_time_since_last_pose_update = rospy.get_param(
    f'/{node_name}/publish_time_since_last_pose_update', True)
publish_poses = rospy.get_param(f'/{node_name}/publish_poses', True)
publish_velocities = rospy.get_param(f'/{node_name}/publish_velocities', True)
publish_accelerations_predicted = rospy.get_param(f'/{node_name}/publish_accelerations_predicted', True)
auto_update_pose = rospy.get_param(f'/{node_name}/auto_update_pose', True)
auto_update_vel = rospy.get_param(f'/{node_name}/auto_update_vel', True)
auto_update_adaptive_tau = rospy.get_param(f'/{node_name}/update_adaptive_tau', True)
tau_update_kalman_k = rospy.get_param(f'/{node_name}/tau_update_kalman_k', 1.0)
acceleration_predictor = rospy.get_param(f'/{node_name}/acceleration_predictor', True)
use_gui = rospy.get_param(f'/{node_name}/use_gui', True)
use_gpu = rospy.get_param(f'/{node_name}/use_gpu', True)
info_simulation_fps = rospy.get_param(f'/{node_name}/info_simulation_fps', True)
observer_rate = rospy.get_param(f'/{node_name}/rate', 220)

if use_gpu:
    Physics.init_gpu()

if publish_time_since_last_pose_update \
        or publish_poses \
        or publish_velocities \
        or publish_accelerations_predicted:
    states_pub = rospy.Publisher(f'/{node_name}/states', MultiDOFJointTrajectoryPoint, queue_size=1)


def setpoint_raw_cb(msg, callback_args):
    global control_mode_velocities
    n, suff = callback_args
    data[n][suff] = msg
    wanted_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                  PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
    control_mode_velocities = ((wanted_mask & msg.type_mask) == wanted_mask)


def pose_topic_cb(msg, callback_args):
    n, suff = callback_args
    data[n][suff] = msg
    if auto_update_pose:
        actors[n].set_global_pose([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])


def vel_topic_cb(msg, callback_args):
    n, suff = callback_args
    data[n][suff] = msg
    velocity = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
    if auto_update_vel:
        actors[n].set_linear_velocity(velocity)
    if acceleration_predictor and auto_update_adaptive_tau:
        update_adaptive_tau(velocity)


def subscribe_on_topics():
    for n in range(1, instances_num + 1):
        suff = "local_position/pose"
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, PoseStamped, pose_topic_cb, callback_args=(n, suff))
    for n in range(1, instances_num + 1):
        suff = "local_position/velocity_local"
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, TwistStamped, vel_topic_cb, callback_args=(n, suff))
    for n in range(1, instances_num + 1):
        suff = "setpoint_raw/local"
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, PositionTarget, setpoint_raw_cb, callback_args=(n, suff))


def spawn_drones(radius):
    for n in range(1, instances_num + 1):
        actor = RigidDynamic()
        shape = Shape.create_sphere(radius, Material(restitution=0.0, dynamic_friction=0.0, static_friction=0.0))
        shape.set_flag(SIMULATION_SHAPE, False)
        actor.attach_shape(shape)
        actor.disable_gravity()
        actor.set_mass(1.)
        actor.set_max_angular_velocity(0)
        actors[n] = actor
        scene.add_actor(actor)


tau_sec_ticks = 1000
tau_min = -15.0
tau_max = 15.0
global tau_func
tau_func = []


def read_adaptive_tau():
    pass


def update_adaptive_tau(velocity):
    # print(velocity)
    pass


def get_adaptive_acceleration(error):
    return copysign(pow(abs(error), 1 / 2), error)


def acceleration_predict():
    global control_mode_velocities
    for n in range(1, instances_num + 1):
        data[n]["acceleration_predicted"] = [0.0, 0.0, 0.0]
        if control_mode_velocities:
            pass
            # TODO: Сделать предсказание ускорения {d3dx13}
            actor_velocity = actors[n].get_linear_velocity()
            control_velocity = data[n].get("setpoint_raw/local")
            """
            if control_velocity is None:
                continue
            control_velocity = control_velocity.velocity
            control_velocity = [control_velocity.x, control_velocity.y, control_velocity.z]
            acceleration_predicted = []
            for i in range(3):
                error = control_velocity[i] - actor_velocity[i]
                acceleration_predicted.append(get_adaptive_acceleration(error))
            data[n]["acceleration_predicted"] = acceleration_predicted
            """
    for n in range(1, instances_num + 1):
        actors[n].add_force(force=data[n]["acceleration_predicted"])


def publish_observer():
    states = MultiDOFJointTrajectoryPoint()
    acceleration_predicted = {}
    for n in range(1, instances_num + 1):
        acceleration_predicted[n] = Twist()
    if publish_time_since_last_pose_update:
        for n in range(1, instances_num + 1):
            last_time = data[n]["local_position/pose"].header.stamp
            last_time = 1.0 * last_time.secs + 1E-9 * last_time.nsecs
            cur_time = rospy.get_time()
            acceleration_predicted[n].angular.x = last_time
            acceleration_predicted[n].angular.y = cur_time
            acceleration_predicted[n].angular.z = cur_time - last_time
    if publish_poses:
        for n in range(1, instances_num + 1):
            pose = Transform()
            actor_pose = actors[n].get_global_pose()[0]
            pose.translation.x = actor_pose[0]
            pose.translation.y = actor_pose[1]
            pose.translation.z = actor_pose[2]
            states.transforms.append(pose)
    if publish_velocities:
        for n in range(1, instances_num + 1):
            velocity = Twist()
            actor_velocity = actors[n].get_linear_velocity()
            velocity.linear.x = actor_velocity[0]
            velocity.linear.y = actor_velocity[1]
            velocity.linear.z = actor_velocity[2]
            states.velocities.append(velocity)
    if publish_accelerations_predicted and acceleration_predictor:
        for n in range(1, instances_num + 1):
            accel_pred = data[n]["acceleration_predicted"]
            acceleration_predicted[n].linear.x = accel_pred[0]
            acceleration_predicted[n].linear.y = accel_pred[1]
            acceleration_predicted[n].linear.z = accel_pred[2]
    if publish_time_since_last_pose_update or \
            (publish_accelerations_predicted and acceleration_predictor):
        for n in range(1, instances_num + 1):
            states.accelerations.append(acceleration_predicted[n])
    states.time_from_start = rospy.get_rostime()
    states_pub.publish(states)


def main():
    rospy.init_node(node_name)

    read_adaptive_tau()
    spawn_drones(0.5)
    subscribe_on_topics()
    sleep(0.2)

    rate = Rate(observer_rate)

    if use_gui:
        render = PyPhysxViewer(wait_for_open=True)
        render.add_physx_scene(scene)
    if info_simulation_fps:
        last_frames_time = time()
        frame_counter = 0

    rospy.loginfo(node_name + " started")

    while True:
        scene.simulate(rate.period())
        if acceleration_predictor:
            acceleration_predict()
        publish_observer()
        if use_gui:
            render.update()
        if info_simulation_fps:
            frame_counter += 1
            if time() - last_frames_time > 5:
                rospy.loginfo(f"fps: {frame_counter / 5}")
                frame_counter = 0
                last_frames_time = time()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

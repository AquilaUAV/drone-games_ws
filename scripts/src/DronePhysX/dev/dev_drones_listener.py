from pyphysx import *
from pyphysx_utils.rate import Rate
from pyphysx_render.pyrender import PyPhysxViewer
from pyphysx_render.meshcat_render import MeshcatViewer
from time import time
from random import randint, random
import numpy as np
import re
import rospy
from time import sleep

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Int64, Bool
from geometry_msgs.msg import TwistStamped, PoseStamped, QuaternionStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped

from tf_conversions.posemath import transformations

Physics.init_gpu()

scene = Scene(scene_flags=[ENABLE_GPU_DYNAMICS, ], )


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
global auto_update_pose
global auto_update_vel
auto_update_pose = False
auto_update_vel = False


def pose_topic_cb(msg, callback_args):
    global auto_update_pose
    n, suff = callback_args
    data[n][suff] = msg
    if drones_pose_auto_update_pose:
        actors[n].set_global_pose([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])


def vel_topic_cb(msg, callback_args):
    global auto_update_vel
    n, suff = callback_args
    data[n][suff] = msg
    if drones_pose_auto_update_vel:
        actors[n].set_linear_velocity([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])


def subscribe_on_topics():
    for n in range(1, instances_num + 1):
        data[n] = {}
        suff = "local_position/pose"
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, PoseStamped, pose_topic_cb, callback_args=(n, suff))
    for n in range(1, instances_num + 1):
        data[n] = {}
        suff = "local_position/velocity_local"
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, TwistStamped, vel_topic_cb, callback_args=(n, suff))


def spawn_drones(radius):
    for n in range(1, instances_num + 1):
        actor = RigidDynamic()
        shape = Shape.create_sphere(radius, Material(restitution=0.0, dynamic_friction=0.0, static_friction=0.0))
        shape.set_flag(SIMULATION_SHAPE, False)
        actor.attach_shape(shape)
        actor.disable_gravity()
        actor.set_mass(1.0)
        actor.set_max_angular_velocity(0)
        actors[n] = actor
        scene.add_actor(actor)


def create_border(L, size):
    actor = RigidStatic()
    actor.attach_shape(Shape.create_box([size, 2 * L + size, L], Material(restitution=1.0)))
    actor.set_global_pose([L, 0, 0])
    scene.add_actor(actor)
    actor = RigidStatic()
    actor.attach_shape(Shape.create_box([size, 2 * L + size, L], Material(restitution=1.0)))
    actor.set_global_pose([-L, 0, 0])
    scene.add_actor(actor)
    actor = RigidStatic()
    actor.attach_shape(Shape.create_box([2 * L + size, size, L], Material(restitution=1.0)))
    actor.set_global_pose([0, L, 0])
    scene.add_actor(actor)
    actor = RigidStatic()
    actor.attach_shape(Shape.create_box([2 * L + size, size, L], Material(restitution=1.0)))
    actor.set_global_pose([0, -L, 0])
    scene.add_actor(actor)
    actor = RigidStatic()
    actor.attach_shape(Shape.create_box([2 * L + size, 2 * L + size, size], Material(restitution=1.0)))
    actor.set_global_pose([0, 0, 0])
    scene.add_actor(actor)


# create_border(50, 10)

# render = PyPhysxViewer(wait_for_open=True, open_meshcat=True)  # video_filename='videos/01_free_fall.gif'


if __name__ == '__main__':
    node_name = "dev_drones_listener"
    rospy.init_node(node_name)
    rospy.loginfo(node_name + " started")

    spawn_drones(0.5)
    subscribe_on_topics()
    sleep(1)

    auto_update_pose = True
    auto_update_vel = True

    render = PyPhysxViewer(wait_for_open=True)
    render.add_physx_scene(scene)

    rate = Rate(220)
    last_frames_time = time()
    frame_counter = 0

    while render.is_active:
        scene.simulate(rate.period())
        render.update()
        frame_counter += 1
        if time() - last_frames_time > 5:
            print(f"fps: {frame_counter / 5}")
            frame_counter = 0
            last_frames_time = time()
        rate.sleep()

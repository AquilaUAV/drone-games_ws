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
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from __init__ import radius


class MultirotorObserver(object):
    def __init__(self, use_gpu=True, render=None, rate=220, node_name="multirotor_observer"):
        self.node_name = node_name
        self.use_gpu = use_gpu
        self.render = render
        self.rate = Rate(rate)
        self.radius = rospy.get_param(f"/{self.node_name}/radius", radius)
        self.scene_flags = []
        self.busy = False
        self.instances_num = self._get_num_drones()
        self.actors = {}
        self.data = {}
        for n in range(1, self.instances_num + 1):
            self.data[n] = {}
        self.control_mode_velocities = False
        self.publish_time_since_last_pose_update = rospy.get_param(
            f'/{self.node_name}/publish_time_since_last_pose_update', True)
        self.publish_poses = rospy.get_param(f'/{self.node_name}/publish_poses', True)
        self.publish_velocities = rospy.get_param(f'/{self.node_name}/publish_velocities', True)
        self.publish_accelerations_predicted = rospy.get_param(f'/{self.node_name}/publish_accelerations_predicted',
                                                               False)
        self.auto_update_pose = rospy.get_param(f'/{self.node_name}/auto_update_pose', True)
        self.auto_update_vel = rospy.get_param(f'/{self.node_name}/auto_update_vel', True)
        self.acceleration_predictor = rospy.get_param(f'/{self.node_name}/acceleration_predictor', False)
        self.info_simulation_fps = rospy.get_param(f'/{self.node_name}/info_simulation_fps', True)
        self.render = None
        if render is not None:
            self.render = render(wait_for_open=True, open_meshcat=True)
        if self.use_gpu:
            Physics.init_gpu()
        self.scene = Scene() if not self.use_gpu else Scene(
            scene_flags=[SceneFlag.ENABLE_PCM, SceneFlag.ENABLE_STABILIZATION, ],
            broad_phase_type=BroadPhaseType.GPU,
            gpu_max_num_partitions=8, gpu_dynamic_allocation_scale=8.,
        )

    def _get_num_drones(self):
        drones_found = set()
        p = re.compile(r"/mavros(.*)/local_position/pose")
        for topic in rospy.get_published_topics():
            for element in p.findall(topic[0]):
                drones_found.add(element)
        return len(drones_found)

    def _setpoint_raw_cb(self, msg, callback_args):
        n, suff = callback_args
        self.data[n][suff] = msg
        wanted_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                      PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        self.control_mode_velocities = ((wanted_mask & msg.type_mask) == wanted_mask)

    def _pose_topic_cb(self, msg, callback_args):
        n, suff = callback_args
        self.data[n][suff] = msg
        if self.auto_update_pose:
            self.actors[n].set_global_pose([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def _vel_topic_cb(self, msg, callback_args):
        n, suff = callback_args
        self.data[n][suff] = msg
        velocity = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
        if self.auto_update_vel:
            self.actors[n].set_linear_velocity(velocity)

    def _subscribe_on_topics(self):
        for n in range(1, self.instances_num + 1):
            suff = "local_position/pose"
            topic = f"/mavros{n}/{suff}"
            rospy.Subscriber(topic, PoseStamped, self._pose_topic_cb, callback_args=(n, suff))
        for n in range(1, self.instances_num + 1):
            suff = "local_position/velocity_local"
            topic = f"/mavros{n}/{suff}"
            rospy.Subscriber(topic, TwistStamped, self._vel_topic_cb, callback_args=(n, suff))
        for n in range(1, self.instances_num + 1):
            suff = "setpoint_raw/local"
            topic = f"/mavros{n}/{suff}"
            rospy.Subscriber(topic, PositionTarget, self._setpoint_raw_cb, callback_args=(n, suff))

    def _spawn_drones(self, radius):
        for n in range(1, self.instances_num + 1):
            actor = RigidDynamic()
            shape = Shape.create_sphere(radius, Material(restitution=0.0, dynamic_friction=0.0, static_friction=0.0))
            shape.set_flag(ShapeFlag.SIMULATION_SHAPE, False)
            actor.attach_shape(shape)
            actor.disable_gravity()
            actor.set_mass(1.)
            actor.set_max_angular_velocity(0)
            self.actors[n] = actor
            self.scene.add_actor(actor)
        if self.render is not None:
            self.render.add_physx_scene(self.scene)

    def _create_publishers(self):
        self.states_pub = rospy.Publisher(f'/{self.node_name}/states', MultiDOFJointTrajectoryPoint, queue_size=1)

    def _acceleration_predict(self):
        for n in range(1, self.instances_num + 1):
            self.data[n]["acceleration_predicted"] = [0.0, 0.0, 0.0]
            if self.control_mode_velocities:
                pass
                # TODO: Сделать предсказание ускорения {d3dx13}
                actor_velocity = self.actors[n].get_linear_velocity()
                control_velocity = self.data[n].get("setpoint_raw/local")
        for n in range(1, self.instances_num + 1):
            self.actors[n].add_force(force=self.data[n]["acceleration_predicted"])

    def _publish_observer(self):
        states = MultiDOFJointTrajectoryPoint()
        acceleration_predicted = {}
        for n in range(1, self.instances_num + 1):
            acceleration_predicted[n] = Twist()
        if self.publish_time_since_last_pose_update:
            for n in range(1, self.instances_num + 1):
                last_time = self.data[n]["local_position/pose"].header.stamp
                last_time = 1.0 * last_time.secs + 1E-9 * last_time.nsecs
                cur_time = rospy.get_time()
                acceleration_predicted[n].angular.x = last_time
                acceleration_predicted[n].angular.y = cur_time
                acceleration_predicted[n].angular.z = cur_time - last_time
        if self.publish_poses:
            for n in range(1, self.instances_num + 1):
                pose = Transform()
                actor_pose = self.actors[n].get_global_pose()[0]
                pose.translation.x = actor_pose[0]
                pose.translation.y = actor_pose[1]
                pose.translation.z = actor_pose[2]
                states.transforms.append(pose)
        if self.publish_velocities:
            for n in range(1, self.instances_num + 1):
                velocity = Twist()
                actor_velocity = self.actors[n].get_linear_velocity()
                velocity.linear.x = actor_velocity[0]
                velocity.linear.y = actor_velocity[1]
                velocity.linear.z = actor_velocity[2]
                states.velocities.append(velocity)
        if self.publish_accelerations_predicted and self.acceleration_predictor:
            for n in range(1, self.instances_num + 1):
                accel_pred = self.data[n]["acceleration_predicted"]
                acceleration_predicted[n].linear.x = accel_pred[0]
                acceleration_predicted[n].linear.y = accel_pred[1]
                acceleration_predicted[n].linear.z = accel_pred[2]
        if self.publish_time_since_last_pose_update or \
                (self.publish_accelerations_predicted and self.acceleration_predictor):
            for n in range(1, self.instances_num + 1):
                states.accelerations.append(acceleration_predicted[n])
        states.time_from_start = rospy.get_rostime()
        self.states_pub.publish(states)

    def _multirotor_observer_loop(self):
        frame_counter = 0
        last_frames_time = time()
        while True:
            self.scene.simulate(self.rate.period())
            if self.acceleration_predictor:
                self._acceleration_predict()
            self._publish_observer()
            if self.render is not None:
                self.render.update()
            if self.info_simulation_fps:
                frame_counter += 1
                if time() - last_frames_time > 5:
                    rospy.loginfo(f"{self.node_name} fps: {frame_counter / 5}")
                    frame_counter = 0
                    last_frames_time = time()
            self.rate.sleep()

    def main(self):

        rospy.init_node(self.node_name)

        self._spawn_drones(self.radius)
        self._subscribe_on_topics()
        self._create_publishers()
        sleep(0.2)

        rospy.loginfo(f"{self.node_name} started")

        try:
            self._multirotor_observer_loop()
        except rospy.ROSInterruptException:
            pass

        rospy.spin()


if __name__ == '__main__':
    multirotor_observer = MultirotorObserver()  # (render=MeshcatViewer or render=PyPhysxViewer)
    multirotor_observer.main()

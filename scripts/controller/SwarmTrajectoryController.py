#!/usr/bin/env python3
# coding=utf8

import numpy as np
from numpy.linalg import linalg
import rospy
from MultirotorController import MultirotorController
from geometry_msgs.msg import PoseArray, Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import Float64, Int64
from math import *


class SwarmTrajectoryController(MultirotorController):
    def __init__(self, freq=20, node_name="swarm_trajectory_controller", takeoff_timeout=1.0):
        super().__init__(freq, node_name, takeoff_timeout)
        self.instances_num = self._get_num_drones()
        self.initial_poses_data = {}
        self.initial_poses_samples = {}
        self.max_poses_errors = 0.5
        self.step_size = 1
        self.pose_error_ku = 3.0
        self.pose_error_windup = 2.0
        self.near_point_border = 2.0

    def estimate_initial_poses(self, n, dt):
        if dt < self.takeoff_timeout / 2:
            if self._data[n].get("local_position/pose") is None:
                return
            if self.initial_poses_data.get(n) is None or self.initial_poses_samples.get(n) is None:
                self.initial_poses_data[n] = np.array([0.0, 0.0, 0.0])
                self.initial_poses_samples[n] = 0
            pose = self._data[n]["local_position/pose"].pose.position
            self.initial_poses_data[n] += np.array([pose.x, pose.y, pose.z])
            self.initial_poses_samples[n] += 1
        elif dt < self.takeoff_timeout:
            initial_poses = PoseArray()
            for n in range(1, self.instances_num + 1):
                pose = Pose()
                initial_pose = self.initial_poses_data[n] / self.initial_poses_samples[n]
                pose.position.x = initial_pose[0]
                pose.position.y = initial_pose[1]
                pose.position.z = initial_pose[2]
                initial_poses.poses.append(pose)
            self.pub_initial_poses.publish(initial_poses)

    def estimate_poses_errors(self):
        result = {}
        for n in range(1, self.instances_num + 1):
            target = self._data[n].get("target_state")
            pose = self._data[n].get("local_position/pose")
            if target is None or pose is None:
                continue
            pose = pose.pose.position
            pose = np.array([pose.x, pose.y, pose.z])
            target = np.array(target)
            result[n] = linalg.norm(target - pose)
        return result

    def create_publishers(self):
        self.pub_initial_poses = rospy.Publisher("/swarm_trajectory_planner/set_initial_poses", PoseArray, queue_size=1)
        self.pub_distance_between_drones = rospy.Publisher("/swarm_trajectory_planner/set_distance_between_drones",
                                                           Float64, queue_size=1)
        self.pub_apply_step = rospy.Publisher("/swarm_trajectory_planner/apply_step", Int64, queue_size=1)

    def update_target_poses(self, msg):
        for n in range(1, self.instances_num + 1):
            self._data[n]["target_state"] = [msg.transforms[n - 1].translation.x,
                                             msg.transforms[n - 1].translation.y,
                                             msg.transforms[n - 1].translation.z]

    def update_states(self, msg):
        for n in range(1, self.instances_num + 1):
            pose = msg.transforms[n - 1].translation
            self._data[n]["inner_state"] = [pose.x, pose.y, pose.z]

    def subscribe_on_topics(self):
        rospy.Subscriber("/swarm_trajectory_planner/target_states", MultiDOFJointTrajectoryPoint,
                         self.update_target_poses)
        # rospy.Subscriber("/multirotor_observer/states", MultiDOFJointTrajectoryPoint, self.update_states)

    def control_raw(self, pt, n, dt):

        self.mc_takeoff(pt, n, dt)
        self.estimate_initial_poses(n, dt)

        point = self._data[n].get("target_state")
        pose = self._data[n].get("local_position/pose")
        if point is not None and pose is not None:
            pose = pose.pose.position
            pose = [pose.x, pose.y, pose.z]
            pose = np.array(pose)
            point = np.array(point)
            vector = point - pose
            if linalg.norm(vector) < self.near_point_border:
                error = self.pose_error_ku * linalg.norm(vector) * exp(linalg.norm(vector))
                if error > self.pose_error_windup:
                    error = self.pose_error_windup
                vector = error * vector / linalg.norm(vector)
                self.set_pos(pt, *(pose + vector).tolist())
                # self.set_pos(pt, *(point).tolist())
            else:
                self.set_pos(pt, *(point).tolist())

        poses_errors = self.estimate_poses_errors()
        if len(poses_errors) > 0 and max(poses_errors.values()) < self.max_poses_errors:
            step = Int64()
            step.data = self.step_size
            self.pub_apply_step.publish(step)


if __name__ == '__main__':
    controller = SwarmTrajectoryController()
    controller.main()

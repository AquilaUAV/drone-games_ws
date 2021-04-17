#!/usr/bin/env python3
# coding=utf8

import numpy as np
from numpy.linalg import linalg
import rospy
from MultirotorController import MultirotorController
from geometry_msgs.msg import PoseArray, Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import Float64


class StepLineTrajectoryController(MultirotorController):
    def __init__(self, freq=220, node_name="step_line_trajectory_controller", takeoff_timeout=1.0):
        super().__init__(freq, node_name, takeoff_timeout)
        self.instances_num = self._get_num_drones()
        self.initial_poses_data = {}
        self.initial_poses_samples = {}
        self.max_poses_errors = 4.0  # МЕНЯТЬ ЭТО
        self.pose_error_ku = 3.0  # МЕНЯТЬ ЭТО

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
            pose = self._data[n].get("inner_state")
            last = self._data[n].get("last_target_state")
            if target is None or pose is None or last is None:
                result[n] = 0.0
                continue
            target = np.array(target)
            pose = np.array(pose)
            last = np.array(last)
            move = target - pose
            vector = target - last
            if linalg.norm(vector) > 0.0:
                vector /= linalg.norm(vector)
            error = move.dot(vector)
            if error <= 0.0:
                error = 0.0
            result[n] = error
        return result

    def create_publishers(self):
        self.pub_initial_poses = rospy.Publisher("/line_trajectory_planner/set_initial_poses", PoseArray, queue_size=1)
        self.pub_distance_between_drones = rospy.Publisher("/line_trajectory_planner/set_distance_between_drones",
                                                           Float64, queue_size=1)
        self.pub_apply_step = rospy.Publisher("/line_trajectory_planner/apply_step", Float64, queue_size=1)

    def update_target_poses(self, msg):
        for n in range(1, self.instances_num + 1):
            self._data[n]["last_target_state"] = self._data[n].get("target_state")
            self._data[n]["target_state"] = [msg.poses[n - 1].position.x, msg.poses[n - 1].position.y,
                                             msg.poses[n - 1].position.z]

    def update_states(self, msg):
        for n in range(1, self.instances_num + 1):
            pose = msg.transforms[n - 1].translation
            self._data[n]["inner_state"] = [pose.x, pose.y, pose.z]

    def subscribe_on_topics(self):
        rospy.Subscriber("/line_trajectory_planner/target_states", PoseArray, self.update_target_poses)
        rospy.Subscriber("/multirotor_observer/states", MultiDOFJointTrajectoryPoint, self.update_states)

    def control_raw(self, pt, n, dt):

        self.mc_takeoff(pt, n, dt)
        self.estimate_initial_poses(n, dt)
        point = self._data[n].get("target_state")
        if point is not None:
            pose = self._data[n].get("inner_state")
            if pose is None:
                return
            pose = np.array(pose)
            vector = np.array(point) - pose
            vector = self.pose_error_ku * vector
            self.set_pos(pt, *(pose + vector).tolist())

        if dt < 2.0:
            step = Float64()
            step.data = 1 / 220
            self.pub_apply_step.publish(step)

        poses_errors = self.estimate_poses_errors()
        if len(poses_errors) > 0 and max(poses_errors.values()) < self.max_poses_errors:
            step = Float64()
            step.data = 1 / 220 * 12
            self.pub_apply_step.publish(step)


if __name__ == '__main__':
    controller = StepLineTrajectoryController()
    controller.main()

#!/usr/bin/env python3
# coding=utf8

from AbstractTrajectoryPlanner import AbstractTrajectoryPlanner
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64, Bool
import rospy
import numpy as np
from numpy import linalg
from math import *


class StepLineTrajectoryPlanner(AbstractTrajectoryPlanner):
    def __init__(self):
        super().__init__(node_name="line_trajectory_planner")
        self.first_step = -2.0
        rospy.loginfo(f"first_step initialized with {self.first_step}")
        self.distance_between_drones = 2.0
        rospy.loginfo(f"distance_between_drones initialized with {self.distance_between_drones}")
        self.initial_poses = None
        self.maximum_start_vectors_lengths = {}
        self.border_size = 10.0
        self.forward_throw = 10.0
        self.backward_throw = 10.0
        self.takeoff_height = 1.0
        self.obstacles_avoided = {}
        self.trajectories = {}
        self.disarm = False
        for n in range(self.instances_num):
            self.trajectories[n] = []
        self.trajectory_created = set({})

    def set_initial_poses(self, msg):
        poses = msg.poses
        poses = [[pose.position.x, pose.position.y, pose.position.z] for pose in poses]
        self.initial_poses = poses
        rospy.loginfo(f"initial_poses is now {self.initial_poses}")

    def set_distance_between_drones(self, msg):
        self.distance_between_drones = msg.data
        rospy.loginfo(f"step_size is now {self.distance_between_drones}")

    def trajectories_update(self):
        if self.initial_poses is None:
            return
        if 0 not in self.trajectory_created:
            for n in range(self.instances_num):
                self.trajectories[n].append(self.initial_poses[n])
            self.trajectory_created.add(0)
        if -2 not in self.trajectory_created and -1 in self.trajectory_created and self._central.get(-1) is not None:
            final_poses = self.calc_final_poses(self._central.get(-1), land_height=-0.1)
            for n in range(self.instances_num):
                self.trajectories[n].append(final_poses[n])
            self.trajectory_created.add(-2)

        keys = self.get_central_keys()
        if keys is None:
            return

        for key in keys:
            if key in self.trajectory_created:
                continue
            for n in range(self.instances_num):
                curent_central = self._central.get(key)
                for i in range(len(curent_central) - 1):
                    move_start = np.array(curent_central[i])
                    move_vector = np.array(curent_central[i + 1]) - move_start
                    throw_vector = None

                    obstacle_avoid_vector = np.array([0.0, 0.0, 0.0])
                    if i == len(curent_central) - 2:
                        obstacle_vector_y = np.array([0.0, 0.0, 1.0])
                        obstacle_vector_x = np.cross(move_vector, obstacle_vector_y)
                        obstacle_vector_x /= linalg.norm(obstacle_vector_x)
                        obstacle_vector_y /= linalg.norm(obstacle_vector_y)
                        vector_free_x, vector_free_y = self.obstacles_avoid(key)
                        obstacle_avoid_vector = obstacle_vector_x * vector_free_x + obstacle_vector_y * vector_free_y
                    if key == 1 and i == 0:
                        move_start = move_start + move_vector * self.border_size / linalg.norm(move_vector)
                    elif key == -1 and i == len(curent_central) - 2:
                        move_vector = move_vector - move_vector * self.border_size / linalg.norm(move_vector)
                    if key != -1 and i == len(curent_central) - 2:
                        move_vector = move_vector - move_vector * self.backward_throw / linalg.norm(move_vector)
                        throw_vector = move_vector * (self.forward_throw + self.backward_throw) / linalg.norm(
                            move_vector)
                    if key == 1:
                        self.trajectories[n].append(move_start.tolist())
                    self.trajectories[n].append((move_start + move_vector + obstacle_avoid_vector).tolist())
                    if throw_vector is not None:
                        self.trajectories[n].append(
                            (move_start + move_vector + throw_vector + obstacle_avoid_vector).tolist())
            self.trajectory_created.add(key)


    def _subscribe_on_topics(self):
        rospy.Subscriber(f"/{self.node_name}/set_initial_poses", PoseArray, self.set_initial_poses)
        rospy.Subscriber(f"/{self.node_name}/set_distance_between_drones", Float64, self.set_distance_between_drones)
        rospy.Subscriber(f"/{self.node_name}/apply_step", Float64, self.apply_step)

    def _create_publishers(self):
        self.pub_target_states = rospy.Publisher(f"/{self.node_name}/target_states", PoseArray, queue_size=1)
        self.pub_disarm = rospy.Publisher("/step_line_trajectory_controller/disarm", Bool, queue_size=1)

    def _planner_loop(self):
        rate = rospy.Rate(220)

        while not rospy.is_shutdown():
            if self.disarm:
                self.pub_disarm.publish(Bool(data=True))
                break
            self.trajectories_update()
            rate.sleep()


if __name__ == '__main__':
    planner = StepLineTrajectoryPlanner()
    planner.main()

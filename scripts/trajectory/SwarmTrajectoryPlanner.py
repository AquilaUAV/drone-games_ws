#!/usr/bin/env python3
# coding=utf8

from FreeSpaceFinder2D import FreeSpaceFinder2D
from AbstractTrajectoryPlanner import AbstractTrajectoryPlanner
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64, Int64, Bool
import rospy
from __init__ import radius
import numpy as np
from numpy import linalg
from math import *


class SwarmTrajectoryPlanner(AbstractTrajectoryPlanner):
    def __init__(self, rate=30):
        super().__init__(node_name="swarm_trajectory_planner")
        self.initial_poses = None
        self.disarm = False
        self.instances_num = None
        self.rate = rate
        self.step = 0
        self.border_size = 10.0
        self.throw_reserve_points = 32
        self.forward_throw = 5.0
        self.backward_throw = 5.0
        self.radius = radius
        self.distance_between_drones = 2.0
        self.optimal_shape_size = radius * 3
        self.optimal_shape_step = 0.0
        self.trajectories = MultiDOFJointTrajectory()
        self.trajectories.header.frame_id = "map"
        self.free_space_finder = FreeSpaceFinder2D()
        self.trajectory_requested = set({})
        self.trajectory_created = set()
        self.trajectoty_version = [1, 0]
        rospy.loginfo(f"distance_between_drones initialized with {self.distance_between_drones}")

    def _transform_from_point(self, point):
        transform = Transform()
        transform.translation.x = point[0]
        transform.translation.y = point[1]
        transform.translation.z = point[2]
        return transform

    def _get_optimal_move_shape(self, point):
        points = []
        if self.initial_poses is None:
            return points
        point = np.array(point)
        size = int(ceil(pow(self.instances_num, 1 / 3)))
        instances_counter = 0
        x_max = 0
        y_max = 0
        z_max = 0
        for z_num in range(size):
            if instances_counter >= self.instances_num:
                break
            for x_num in range(size):
                if instances_counter >= self.instances_num:
                    break
                for y_num in range(size):
                    if instances_counter >= self.instances_num:
                        break
                    points.append(np.array([x_num * self.optimal_shape_size,
                                            y_num * self.optimal_shape_size,
                                            z_num * self.optimal_shape_size]))
                    instances_counter += 1
                    x_max = max(x_num + 1, x_max)
                    y_max = max(y_num + 1, y_max)
                    z_max = max(z_num + 1, z_max)
        for num in range(len(points)):
            points[num] += point
            if x_max > 1:
                points[num] -= np.array(
                    [(x_max - 1) * self.optimal_shape_size / 2, 0, 0])
            if y_max > 1:
                points[num] -= np.array(
                    [0, (y_max - 1) * self.optimal_shape_size / 2, 0])
            if z_max > 1:
                points[num] -= np.array(
                    [0, 0, (z_max - 1) * self.optimal_shape_size / 2])
        return points

    def set_initial_poses(self, msg):
        poses = msg.poses
        poses = [[pose.position.x, pose.position.y, pose.position.z] for pose in poses]
        self.initial_poses = poses
        self.instances_num = len(self.initial_poses)
        self.optimal_shape_step = (int(ceil(pow(self.instances_num, 1 / 3))) - 1) * self.optimal_shape_size / 2
        self.optimal_shape_step *= sqrt(2)
        initial_point = MultiDOFJointTrajectoryPoint()
        for point in self.initial_poses:
            initial_point.transforms.append(self._transform_from_point(point))
        self.trajectory_created.add("0.3")
        self.trajectories.points.append(initial_point)
        rospy.loginfo(f"initial_poses is now {self.initial_poses}")

    def set_distance_between_drones(self, msg):
        self.distance_between_drones = msg.data
        rospy.loginfo(f"step_size is now {self.distance_between_drones}")

    def path_planner_cb(self, msg):
        version = msg.header.frame_id.split(".")
        if int(version[0]) == self.trajectoty_version[0] and int(version[1]) == self.trajectoty_version[1]:
            self.trajectories.points.extend(msg.points)
            rospy.loginfo(f"created: {msg.header.frame_id}")
            self.trajectory_created.add(msg.header.frame_id)
            if self.trajectoty_version[1] < 4:
                self.trajectoty_version[1] += 1
            else:
                self.trajectoty_version[0] += 1
                self.trajectoty_version[1] = 0

    def trajectories_update(self):
        if self.initial_poses is None:
            return

        if f"{self.trajectoty_version[0]}.{self.trajectoty_version[1]}" not in self.trajectory_requested:
            if self.trajectoty_version[1] == 0:
                if f"{self.trajectoty_version[0] - 1}.{3}" in self.trajectory_created:
                    central = self._central.get(self.trajectoty_version[0])
                    if central is not None:
                        target = MultiDOFJointTrajectory()
                        target.header.frame_id = f"{self.trajectoty_version[0]}.{self.trajectoty_version[1]}"
                        first_point = MultiDOFJointTrajectoryPoint()
                        second_point = MultiDOFJointTrajectoryPoint()
                        first_point = self.trajectories.points[-1]
                        move_point = np.array(central[0])
                        move_vector = np.array(central[1]) - move_point
                        if self.trajectoty_version[0] == 1:
                            move_vector = move_vector * self.border_size / linalg.norm(move_vector)
                        else:
                            move_vector = move_vector * (
                                    self.forward_throw + self.optimal_shape_step) / linalg.norm(move_vector)
                        for point in self._get_optimal_move_shape((move_point + move_vector).tolist()):
                            second_point.transforms.append(self._transform_from_point(point))
                        target.points.append(first_point)
                        target.points.append(second_point)
                        self.pub_path_planner.publish(target)
                        rospy.loginfo(f"requested: {target.header.frame_id}")
                        self.trajectory_requested.add(target.header.frame_id)
            elif self.trajectoty_version[1] == 1:
                if f"{self.trajectoty_version[0]}.{0}" in self.trajectory_created:
                    central = self._central.get(self.trajectoty_version[0])
                    if central is not None:
                        for i in range(len(central) - 1):
                            old_point = self.trajectories.points[-1]
                            move_point = np.array(central[i])
                            move_vector = np.array(central[i + 1]) - move_point
                            if i == 0:
                                if self.trajectoty_version[0] == 1:
                                    move_point += move_vector * self.border_size / linalg.norm(move_vector)
                                else:
                                    move_point += move_vector * (
                                            self.forward_throw + self.optimal_shape_step) / linalg.norm(move_vector)
                            move_vector = np.array(central[i + 1]) - move_point
                            if i == len(central) - 2:
                                move_vector -= move_vector * (
                                        self.backward_throw + self.optimal_shape_step) / linalg.norm(move_vector)
                            target = MultiDOFJointTrajectoryPoint()
                            for drone in range(len(old_point.transforms)):
                                new_vector = old_point.transforms[drone].translation
                                new_vector = np.array([new_vector.x, new_vector.y, new_vector.z])
                                new_vector = new_vector + move_vector
                                new_vector = self._transform_from_point(new_vector)
                                target.transforms.append(new_vector)
                            for j in range(self.throw_reserve_points):
                                self.trajectories.points.append(target)
                        new_point = f"{self.trajectoty_version[0]}.{self.trajectoty_version[1]}"
                        self.trajectory_requested.add(new_point)
                        self.trajectory_created.add(new_point)
                        rospy.loginfo(f"requested: {new_point}")
                        rospy.loginfo(f"created: {new_point}")
                        self.trajectoty_version[1] += 1
            elif self.trajectoty_version[1] == 2:
                if f"{self.trajectoty_version[0]}.{1}" in self.trajectory_created:
                    walls = self._walls.get(self.trajectoty_version[0])
                    central = self._central.get(self.trajectoty_version[0])
                    if walls is not None and central is not None:
                        target = MultiDOFJointTrajectory()
                        target.header.frame_id = f"{self.trajectoty_version[0]}.{self.trajectoty_version[1]}"
                        first_point = self.trajectories.points[-1]
                        second_point = MultiDOFJointTrajectoryPoint()
                        free_space = self.free_space_finder.find_optimal_points(walls, self.radius, self.instances_num)
                        move_point = np.array(central[len(central) - 2])
                        move_vector = np.array(central[len(central) - 1]) - move_point
                        move_vector -= move_vector * self.backward_throw / linalg.norm(move_vector)
                        final_plane = move_point + move_vector
                        x_free = np.cross(move_vector, np.array([0.0, 0.0, 1.0]))
                        y_free = np.cross(x_free, move_vector)
                        x_free_bias = 0.0
                        y_free_bias = 0.0

                        if self.trajectoty_version[0] == 6:
                            x_free_bias = 1.0

                        x_free /= linalg.norm(x_free)
                        y_free /= linalg.norm(y_free)
                        for step_back in range(self.instances_num // len(free_space) + 1):
                            if step_back != (self.instances_num // len(free_space)):
                                for space in free_space:
                                    point = final_plane + (space[0] + x_free_bias) * x_free + (space[1] + y_free_bias) \
                                            * y_free - move_vector * self.distance_between_drones * step_back \
                                            / linalg.norm(move_vector)
                                    second_point.transforms.append(self._transform_from_point(point))
                            elif self.instances_num % len(free_space) != 0:
                                final_free_space = self.free_space_finder.find_optimal_points(walls, self.radius,
                                                                                              self.instances_num % len(
                                                                                                  free_space))
                                for space in final_free_space:
                                    point = final_plane + (space[0] + x_free_bias) * x_free + (space[1] + y_free_bias) \
                                            * y_free - move_vector * self.distance_between_drones * step_back \
                                            / linalg.norm(move_vector)
                                    second_point.transforms.append(self._transform_from_point(point))
                        target.points.append(first_point)
                        target.points.append(second_point)
                        self.pub_path_planner.publish(target)
                        rospy.loginfo(f"requested: {target.header.frame_id}")
                        self.trajectory_requested.add(target.header.frame_id)
            elif self.trajectoty_version[1] == 3:
                if f"{self.trajectoty_version[0]}.{2}" in self.trajectory_created:
                    central = self._central.get(self.trajectoty_version[0])
                    if central is not None:
                        target = MultiDOFJointTrajectoryPoint()
                        old_point = self.trajectories.points[-1]
                        move_vector = np.array(central[len(central) - 1]) - np.array(central[len(central) - 2])
                        move_vector = move_vector * (
                                self.backward_throw + self.forward_throw + 2 * self.optimal_shape_step) / linalg.norm(
                            move_vector)
                        for drone in range(len(old_point.transforms)):
                            new_vector = old_point.transforms[drone].translation
                            new_vector = np.array([new_vector.x, new_vector.y, new_vector.z])
                            new_vector = new_vector + move_vector
                            new_vector = self._transform_from_point(new_vector)
                            target.transforms.append(new_vector)
                        self.trajectories.points.append(target)
                        new_point = f"{self.trajectoty_version[0]}.{self.trajectoty_version[1]}"
                        self.trajectory_requested.add(new_point)
                        self.trajectory_created.add(new_point)
                        rospy.loginfo(f"requested: {new_point}")
                        rospy.loginfo(f"created: {new_point}")
                        self.trajectoty_version[0] += 1
                        self.trajectoty_version[1] = 0

        if self.step < 0:
            self.step = 0
        len_trajectories = len(self.trajectories.points)
        if self.step >= len_trajectories - 1:
            self.step = len_trajectories - 1
        if len_trajectories > 0:
            self.pub_target_states.publish(self.trajectories.points[self.step])

    def apply_step(self, msg):
        self.step += msg.data

    def _subscribe_on_topics(self):
        rospy.Subscriber(f"/physx_path_planner/trajectory", MultiDOFJointTrajectory, self.path_planner_cb)
        rospy.Subscriber(f"/{self.node_name}/set_initial_poses", PoseArray, self.set_initial_poses)
        rospy.Subscriber(f"/{self.node_name}/set_distance_between_drones", Float64, self.set_distance_between_drones)
        rospy.Subscriber(f"/{self.node_name}/apply_step", Int64, self.apply_step)

    def _create_publishers(self):
        self.pub_path_planner = rospy.Publisher(f"/physx_path_planner/target_vectors", MultiDOFJointTrajectory,
                                                queue_size=1)
        self.pub_target_states = rospy.Publisher(f"/{self.node_name}/target_states", MultiDOFJointTrajectoryPoint,
                                                 queue_size=1)
        self.pub_disarm = rospy.Publisher("/swarm_trajectory_controller/disarm", Bool, queue_size=1)

    def _planner_loop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.disarm:
                self.pub_disarm.publish(Bool(data=True))
                break
            self.trajectories_update()
            rate.sleep()


if __name__ == '__main__':
    planner = SwarmTrajectoryPlanner()
    planner.main()

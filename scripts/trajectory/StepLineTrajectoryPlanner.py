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
        self.distance_between_drones = 2.0  # МЕНЯТЬ ЭТО
        rospy.loginfo(f"first_step initialized with {self.first_step}")
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

    def obstacles_avoid(self, key):
        if key in {-1, 0}:
            return [0.0, 0.0]
        if key == -2:
            return self.obstacles_avoid(max(self._walls.keys()))
        if self.obstacles_avoided.get(key) is not None:
            return self.obstacles_avoided.get(key)
        if key not in self._walls.keys():
            free = [0.0, 0.0]
        else:
            free_spaces = self._walls.get(key)
            if len(self._central.get(key)) > 2:
                previous = [0.0, 0.0]
            else:
                previous = self.obstacles_avoid(key - 1)
            maximum_free_space = max([min(*spaces[2:]) for spaces in free_spaces])
            maximum_spaces = [spaces for spaces in free_spaces if min(*spaces[2:]) == maximum_free_space]
            optimal_lines = [[spaces[0], spaces[1], spaces[2] - maximum_free_space, spaces[3] - maximum_free_space] for
                             spaces in
                             maximum_spaces]
            good_vectors = []
            for line in optimal_lines:
                if line[2] == 0 and line[3] == 0:
                    # Точка
                    center = line[:2]
                    center[0] -= previous[0]
                    center[1] -= previous[1]
                    good_vectors.append(center)
                elif line[2] == 0:
                    # Вертикальная
                    down_y = line[1] - line[3] / 2
                    up_y = line[1] + line[3] / 2
                    both_x = line[0] - previous[0]
                    down_vector = [both_x - previous[0], down_y - previous[1]]
                    up_vector = [both_x - previous[0], up_y - previous[1]]
                    if linalg.norm(down_vector) < linalg.norm(up_vector):
                        good_vector = down_vector
                    else:
                        good_vector = up_vector
                    if down_y <= previous[1] <= up_y and both_x < linalg.norm(good_vector):
                        good_vector = [both_x, 0.0]
                    good_vectors.append(good_vector)
                elif line[3] == 0:
                    # Горизонтальная
                    left_x = line[0] - line[2] / 2
                    right_x = line[0] + line[2] / 2
                    both_y = line[1] - previous[1]
                    left_vector = [left_x - previous[0], both_y - previous[1]]
                    right_vector = [right_x - previous[0], both_y - previous[1]]
                    if linalg.norm(left_vector) < linalg.norm(right_vector):
                        good_vector = left_vector
                    else:
                        good_vector = right_vector
                    if left_x <= previous[0] <= right_x and both_y < linalg.norm(good_vector):
                        good_vector = [0.0, both_y]
                    good_vectors.append(good_vector)
            min_vector_len = min([linalg.norm(np.array(vector)) for vector in good_vectors])
            best_vectors = [vector for vector in good_vectors if linalg.norm(np.array(vector)) == min_vector_len]
            free = best_vectors[0]
            free[0] += previous[0]
            free[1] += previous[1]
        self.obstacles_avoided[key] = free
        return free[0], free[1]

    def calc_final_poses(self, last_central, land_height=0.0):
        final_poses = []
        last_central = last_central[len(last_central) - 2: len(last_central)]
        move_finish = np.array(last_central[1])
        move_vector = move_finish - np.array(last_central[0])
        move_finish = move_finish - move_vector * self.border_size / linalg.norm(move_vector)
        move_finish[2] = land_height
        land_x = np.cross(move_vector, np.array([0.0, 0.0, 1.0]))
        lanx_y = np.cross(land_x, move_vector)
        land_x /= linalg.norm(land_x)
        lanx_y /= linalg.norm(lanx_y)
        size_x = ceil(sqrt(self.instances_num))
        size_y = ceil(int(self.instances_num) / size_x)
        step_x = 2 * self.border_size / (size_x + 1)
        step_y = 2 * self.border_size / (size_y + 1)
        for i in range(int(self.instances_num)):
            x = i % size_x
            y = i // size_x
            local_land = ((1 + x) * step_x - self.border_size) * land_x + (
                    (1 + y) * step_y - self.border_size) * lanx_y + move_finish
            final_poses.append(local_land.tolist())
        return final_poses

    def get_central_keys(self):
        keys = list(self._central.keys())
        keys.sort()
        if len(keys) == 0:
            return
        keys_neg = [key for key in keys if key < 0]
        keys = [key for key in keys if key >= 0]
        keys.extend(keys_neg[::-1])
        return keys

    def get_maximum_start_vectors_lengths(self, step):
        result = self.maximum_start_vectors_lengths.get(step)
        if result is not None:
            return result
        for n in range(self.instances_num):
            if step + 1 >= len(self.trajectories[n]):
                continue
            length = linalg.norm(np.array(self.trajectories[n][step + 1]) - np.array(self.trajectories[n][step]))
            if result is None or length > result:
                result = length
        if result is not None:
            self.maximum_start_vectors_lengths[step] = result
        return result

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
                        obstacle_vector_x = np.cross(move_vector, np.array([0.0, 0.0, 1.0]))
                        obstacle_vector_y = np.cross(obstacle_vector_x, move_vector)
                        obstacle_vector_x /= linalg.norm(obstacle_vector_x)
                        obstacle_vector_y /= linalg.norm(obstacle_vector_y)
                        vector_free_x, vector_free_y = self.obstacles_avoid(key)

                        x_free_bias = 0.0
                        y_free_bias = 0.0

                        if key == 6:
                            x_free_bias = 1.0

                        obstacle_avoid_vector = obstacle_vector_x * (
                                vector_free_x + x_free_bias) + obstacle_vector_y * (vector_free_y + y_free_bias)
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

    def apply_step(self, msg):
        if self.initial_poses is None:
            rospy.logwarn(f"initial_poses is {self.initial_poses}; Waifing for publishing initial_poses")
            return
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"

        points_amount = min([len(self.trajectories[num]) for num in range(self.instances_num)])
        if points_amount < 2:
            return
        self.first_step += msg.data

        for n in range(self.instances_num):
            target_length = self.first_step - self.distance_between_drones * n

            new_pose = None
            accumulated_length = 0.0

            if target_length < 0.0:
                target_length = 0.0
            for i in range(points_amount - 1):
                if new_pose is not None:
                    break
                if target_length > self.get_maximum_start_vectors_lengths(i) + accumulated_length:
                    accumulated_length += self.get_maximum_start_vectors_lengths(i)
                    continue
                move_start = np.array(self.trajectories[n][i])
                move_vector = np.array(self.trajectories[n][i + 1]) - move_start
                new_pose = move_start + move_vector * (
                        target_length - accumulated_length) / self.get_maximum_start_vectors_lengths(i)
            if new_pose is None:
                new_pose = self.trajectories[n][points_amount - 1]
                if n == self.instances_num - 1:
                    self.disarm = True
            pose = Pose()
            pose.position.x = new_pose[0]
            pose.position.y = new_pose[1]
            pose.position.z = new_pose[2]
            pose_array.poses.append(pose)
        self.pub_target_states.publish(pose_array)

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

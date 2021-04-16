#!/usr/bin/env python3
# coding=utf8

from AbstractTrajectoryPlanner import AbstractTrajectoryPlanner
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64, Int64, Bool
import rospy
import numpy as np
from numpy import linalg
from math import *


class SwarmTrajectoryPlanner(AbstractTrajectoryPlanner):
    def __init__(self):
        super().__init__(node_name="swarm_trajectory_planner")
        self.distance_between_drones = 2.0
        rospy.loginfo(f"distance_between_drones initialized with {self.distance_between_drones}")
        self.initial_poses = None
        self.disarm = False
        self.step = 0
        self.border_size = 10.0
        self.forward_throw = 5.0
        self.backward_throw = 5.0
        self.takeoff_height = 5.0
        self.trajectories = MultiDOFJointTrajectory()
        self.trajectories.header.frame_id = "map"
        self.trajectory_requested = set({})
        self.trajectory_created = set({})

    def _transform_from_point(self, point):
        transform = Transform()
        transform.translation.x = point[0]
        transform.translation.y = point[1]
        transform.translation.z = point[2]
        return transform

    def set_initial_poses(self, msg):
        poses = msg.poses
        poses = [[pose.position.x, pose.position.y, pose.position.z] for pose in poses]
        self.initial_poses = poses
        rospy.loginfo(f"initial_poses is now {self.initial_poses}")

    def set_distance_between_drones(self, msg):
        self.distance_between_drones = msg.data
        rospy.loginfo(f"step_size is now {self.distance_between_drones}")

    def path_planner_cb(self, msg):
        if msg.header.frame_id == "0-0":
            # rospy.logwarn(msg)
            self.trajectories.points.extend(msg.points)
            rospy.logwarn(f"created: 0-0")
            self.trajectory_created.add(msg.header.frame_id)
        pass

    def trajectories_update(self):
        if self.initial_poses is None:
            return
        if "0-0" not in self.trajectory_requested:
            target = MultiDOFJointTrajectory()
            target.header.frame_id = "0-0"
            first_point = MultiDOFJointTrajectoryPoint()
            for point in self.initial_poses:
                first_point.transforms.append(self._transform_from_point(point))
            second_point = MultiDOFJointTrajectoryPoint()
            for point in self.initial_poses:
                point[2] += self.takeoff_height
                second_point.transforms.append(self._transform_from_point(point))
            target.points.append(first_point)
            target.points.append(second_point)
            self.pub_path_planner.publish(target)
            rospy.logwarn(f"requested: 0-0")
            self.trajectory_requested.add("0-0")
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
        rate = rospy.Rate(220)
        while not rospy.is_shutdown():
            if self.disarm:
                self.pub_disarm.publish(Bool(data=True))
                break
            self.trajectories_update()
            rate.sleep()


if __name__ == '__main__':
    planner = SwarmTrajectoryPlanner()
    planner.main()

#!/usr/bin/env python3
# coding=utf8

from pyphysx import *
from pyphysx_utils.rate import Rate
from pyphysx_render.pyrender import PyPhysxViewer
from pyphysx_render.meshcat_render import MeshcatViewer
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from std_msgs.msg import Float64
from time import *
import multiprocessing
import numpy as np
from numpy.linalg import linalg
from collections import deque

from scipy.optimize import linear_sum_assignment
from scipy.sparse.csgraph import maximum_bipartite_matching
from scipy.sparse import csr_matrix

from __init__ import radius


class MultiagentPathPlanner(object):
    def __init__(self, is_2d=False, use_gpu=True, no_sleep=True, render=None, rate=30, node_name="physx_path_planner"):
        self.node_name = node_name
        self.is_2d = is_2d
        self.use_gpu = use_gpu
        self.no_sleep = no_sleep
        self.render = render
        self.rate = Rate(rate)
        self.radius = rospy.get_param(f"/{self.node_name}/radius", radius)
        self.scene_flags = []
        self.actors_velocities = rospy.get_param(f'/{self.node_name}/actors_velocities', 1.00)
        self.actors_final_error = rospy.get_param(f'/{self.node_name}/actors_final_error', 0.01)
        self.actors_error_queue_size = rospy.get_param(f'/{self.node_name}/actors_error_queue_size', 10)
        self.actors_error_queue = deque()
        self.time_limit = rospy.get_param(f'/{self.node_name}/time_limit', 20.0)
        self.busy = False
        self.manager = multiprocessing.Manager()

    def _spawn_actors(self, poses, scene):
        for pose in poses:
            actor = RigidDynamic()
            shape = Shape.create_sphere(self.radius,
                                        Material(restitution=0.0, dynamic_friction=0.0, static_friction=0.0))
            actor.attach_shape(shape)
            actor.set_global_pose(pose=pose)
            actor.disable_gravity()
            actor.set_mass(1.0)
            actor.set_max_angular_velocity(0)
            actor.set_max_linear_velocity(self.actors_velocities)
            scene.add_actor(actor)

    def _calc_final_error(self, current_points, last_points, permutations):
        max_vector = None
        for num in range(len(current_points)):
            vector = np.array(last_points[permutations[num]]) - np.array(current_points[num])
            if max_vector is None or linalg.norm(vector) > max_vector:
                max_vector = linalg.norm(vector)
        return max_vector

    # К какой финальной позиции поедет агент с индексом i
    def _calc_permutations(self, current_points, last_points):
        sign_matrix = np.zeros((len(current_points), len(current_points)), dtype=np.float64)
        ones_matrix = np.ones((len(current_points), len(current_points)), dtype=np.float64)
        possible_distances = []
        for y in range(len(current_points)):
            for x in range(len(last_points)):
                sign_matrix[y][x] = linalg.norm(np.array(current_points[y]) - np.array(last_points[x]))
                possible_distances.append(sign_matrix[y][x])
        possible_distances.sort()
        min_distance_id = 0
        max_distance_id = len(possible_distances) - 1
        while min_distance_id + 1 < max_distance_id:
            mid_distance_id = (max_distance_id - min_distance_id) // 2 + min_distance_id
            test_matrix = csr_matrix((sign_matrix - possible_distances[mid_distance_id] * ones_matrix) <= 0)
            if (maximum_bipartite_matching(test_matrix, perm_type='column') != -1).all():
                max_distance_id = mid_distance_id
            else:
                min_distance_id = mid_distance_id
        test_matrix = (sign_matrix - possible_distances[max_distance_id] * ones_matrix) <= 0
        for y in range(len(current_points)):
            for x in range(len(last_points)):
                if not test_matrix[y][x]:
                    sign_matrix[y][x] = np.inf
        row_ind, col_ind = linear_sum_assignment(sign_matrix)
        return col_ind

    def _update_velocities(self, scene, current_points, last_points, permutations):
        actors = scene.get_dynamic_rigid_actors()
        if self.is_2d:
            for num in range(len(actors)):
                pose = current_points[num]
                pose[2] = 0.0
                actors[num].set_global_pose(pose=pose)
        for num in range(len(actors)):
            velocity_vector = np.array(last_points[permutations[num]]) - np.array(current_points[num])
            velocity = linalg.norm(velocity_vector)
            if velocity != 0.0:
                velocity_vector = velocity_vector / velocity * (
                    min(velocity * self.rate.frequency(), self.actors_velocities))
                actors[num].set_linear_velocity(vel=velocity_vector.tolist())
            else:
                actors[num].set_linear_velocity(vel=[0.0, 0.0, 0.0])

    def _simulation(self, trajectory, start_points, last_points):
        rospy.loginfo("Started path planning")
        start_time = time()
        self.actors_error_queue.clear()

        Physics.init_gpu()
        scene = Scene() if not self.use_gpu else Scene(
            scene_flags=[SceneFlag.ENABLE_PCM, SceneFlag.ENABLE_GPU_DYNAMICS, SceneFlag.ENABLE_STABILIZATION],
            broad_phase_type=BroadPhaseType.GPU,
            gpu_max_num_partitions=1, gpu_dynamic_allocation_scale=1.,
        )

        self._spawn_actors(start_points, scene)

        if self.render is not None:
            render = self.render(wait_for_open=True, open_meshcat=True)
            render.add_physx_scene(scene)

        while time() - start_time < self.time_limit:
            current_points = []
            for actor in scene.get_dynamic_rigid_actors():
                current_points.append(list(actor.get_global_pose()[0]))
            permutations = self._calc_permutations(current_points, last_points)
            self._update_velocities(scene, current_points, last_points, permutations)
            scene.simulate(self.rate.period())
            new_position = []
            for actor in scene.get_dynamic_rigid_actors():
                new_position.append([list(actor.get_global_pose()[0]), list(actor.get_linear_velocity())])
            trajectory.append(new_position)
            if self.render is not None:
                render.update()
            while len(self.actors_error_queue) > self.actors_error_queue_size:
                self.actors_error_queue.pop()
            final_error = self._calc_final_error(current_points, last_points, permutations)
            self.actors_error_queue.append(final_error)
            if len(self.actors_error_queue) > self.actors_error_queue_size:
                final_error_old = self.actors_error_queue.popleft()
                if final_error + self.actors_final_error > final_error_old:
                    break
            if final_error < self.actors_final_error:
                break
            if not self.no_sleep:
                self.rate.sleep()

        rospy.loginfo(
            f"\nMaximum final error is: {final_error}"
            f"\nPath planning took: {time() - start_time} secs"
            f"\nNumber of actors: {len(scene.get_dynamic_rigid_actors())}")

    def _create_simulation(self, msg):
        if self.busy:
            return
        self.busy = True

        points = MultiDOFJointTrajectory()
        points.header.frame_id = msg.header.frame_id
        points.header.stamp = rospy.get_rostime()

        trajectory = self.manager.list()
        start_point_old = msg.points[0].transforms
        start_point = []
        for point in start_point_old:
            start_point.append([point.translation.x, point.translation.y, point.translation.z])
        last_point_old = msg.points[1].transforms
        last_point = []
        for point in last_point_old:
            last_point.append([point.translation.x, point.translation.y, point.translation.z])
        p = multiprocessing.Process(target=self._simulation, args=(trajectory, start_point, last_point,))
        p.start()
        p.join()

        for step in trajectory:
            point = MultiDOFJointTrajectoryPoint()
            for agent in step:
                transform = Transform()
                twist = Twist()
                transform.translation.x = agent[0][0]
                transform.translation.y = agent[0][1]
                transform.translation.z = agent[0][2]
                point.transforms.append(transform)
                twist.linear.x = agent[1][0]
                twist.linear.y = agent[1][1]
                twist.linear.z = agent[1][2]
                point.velocities.append(twist)
            points.points.append(point)

        self.trajectory_pub.publish(points)

        rospy.loginfo(f"Trajectory length: {len(trajectory)}")

        self.busy = False

    def _update_radius(self, msg):
        self.radius = msg.data

    def _subscribe_on_topics(self):
        rospy.Subscriber(f"/{self.node_name}/target_vectors", MultiDOFJointTrajectory, self._create_simulation)
        rospy.Subscriber(f"/{self.node_name}/update_radius", Float64, self._update_radius)

    def _create_publishers(self):
        self.trajectory_pub = rospy.Publisher(f"/{self.node_name}/trajectory", MultiDOFJointTrajectory, queue_size=1)

    def _path_finder_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    def main(self):
        rospy.init_node(self.node_name)
        rospy.loginfo(f"{self.node_name} started")

        self._subscribe_on_topics()
        self._create_publishers()

        try:
            self._path_finder_loop()
        except rospy.ROSInterruptException:
            pass

        rospy.spin()


if __name__ == '__main__':
    path_finder_2d = MultiagentPathPlanner()  # (render=MeshcatViewer, no_sleep=False) or render=PyPhysxViewer
    path_finder_2d.main()

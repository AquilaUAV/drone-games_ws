#!/usr/bin/env python3
# coding=utf8

import rospy
import re
from std_msgs.msg import String


class AbstractTrajectoryPlanner(object):
    def __init__(self, central_iterator_prefix="w", central_marker_finish="|",
                 walls_iterator_prefix="w", node_name="trajectory_planner"):
        self.instances_num = self.__get_num_drones()
        self._central = {}
        self._walls = {}
        self._central_iterator_prefix = central_iterator_prefix
        self._central_marker_finish = central_marker_finish
        self._walls_iterator_prefix = walls_iterator_prefix
        self.node_name = node_name

    def __get_num_drones(self):
        drones_found = set()
        p = re.compile(r"/mavros(.*)/local_position/pose")
        for topic in rospy.get_published_topics():
            for element in p.findall(topic[0]):
                drones_found.add(element)
        return len(drones_found)

    def __subscribe_on_path_generator_topics(self):
        rospy.Subscriber("/path_generator/central", String, self._update_central)
        rospy.Subscriber("/path_generator/walls", String, self._update_walls)

    def _on_shutdown_cb(self):
        rospy.logfatal("shutdown")

    def _update_central(self, central_new):
        if central_new is None:
            return
        central_new = central_new.data.split(' ')
        if central_new[1].startswith(self._central_iterator_prefix):
            num = int(central_new[1][1:])
        elif central_new[1].startswith(self._central_marker_finish):
            num = -1
        else:
            return
        self._central[num] = []
        for i in range(len(central_new[2:]) // 3):
            self._central[num].append([float(n) for n in central_new[(2 + i * 3):(2 + i * 3) + 3]])
        return

    def _update_walls(self, walls_new):
        if walls_new is None:
            return
        walls_new = walls_new.data.split(' ')
        if walls_new[1].startswith(self._walls_iterator_prefix):
            num = int(walls_new[1][1:])
        else:
            return
        self._walls[num] = []
        for i in range(len(walls_new[2:]) // 4):
            self._walls[num].append([float(n) for n in walls_new[(2 + i * 4):(2 + i * 4) + 4]])
        return

    def _create_publishers(self):
        return

    def _subscribe_on_topics(self):
        return

    def _planner_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    def main(self):
        rospy.init_node(self.node_name)
        rospy.loginfo(f"{self.node_name} started with {self.instances_num} drones")

        self.__subscribe_on_path_generator_topics()
        self._subscribe_on_topics()
        self._create_publishers()
        rospy.on_shutdown(self._on_shutdown_cb)

        try:
            self._planner_loop()
        except rospy.ROSInterruptException:
            pass

        rospy.spin()

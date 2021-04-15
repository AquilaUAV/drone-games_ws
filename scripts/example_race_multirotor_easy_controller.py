#!/usr/bin/env python3
# coding=utf8


from controller import MultirotorController
from math import *
from std_msgs.msg import String
import rospy


class MyController(MultirotorController):
    def subscribe_on_control_topics(self):
        rospy.Subscriber("/path_generator/central", String, self._control_cb,
                         callback_args=("/path_generator/central",))
        rospy.Subscriber("/path_generator/walls", String, self._control_cb, callback_args=("/path_generator/walls",))

    def control_user(self, pt, n, dt, walls, central, copter_pose):
        print(walls)
        print(central)
        print(copter_pose)
        self.set_pos(pt, sin(dt + 2 * pi * n / self.instances_num), cos(dt + 2 * pi * n / self.instances_num), n)


controller = MyController()
controller.main()

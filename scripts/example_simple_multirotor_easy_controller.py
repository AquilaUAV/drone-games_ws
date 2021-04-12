#!/usr/bin/env python3
# coding=utf8


from controller import MultirotorController
from math import *
from std_msgs.msg import String
import rospy


class MyController(MultirotorController):
    def control_raw(self, pt, n, dt):
        self.mc_takeoff(pt, n, dt)
        self.set_pos(pt, sin(dt + 2 * pi * n / self.instances_num), cos(dt + 2 * pi * n / self.instances_num), n)


controller = MyController()
controller.main()

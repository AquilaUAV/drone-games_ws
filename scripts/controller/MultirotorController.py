#!/usr/bin/env python3
# coding=utf8

import rospy
from time import time
import re

from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Bool


class MultirotorController(object):
    _data = {}
    walls = {}
    central = {}
    copter_poses = {}

    def __init__(self, freq=20, node_name="multirotor_easy_controller", takeoff_timeout=0.4):
        self._freq = freq
        self.node_name = node_name
        self.takeoff_timeout = takeoff_timeout
        self.instances_num = self._get_num_drones()
        self.t0 = time()
        self.disarm = False

    def _get_num_drones(self):
        drones_found = set()
        p = re.compile(r"/mavros(.*)/local_position/pose")
        for topic in rospy.get_published_topics():
            for element in p.findall(topic[0]):
                drones_found.add(element)
        return len(drones_found)

    def _subscribe_on_mavros_topics(self, suff, data_class):
        for n in range(1, self.instances_num + 1):
            self._data[n] = {}
            topic = f"/mavros{n}/{suff}"
            rospy.Subscriber(topic, data_class, self._topic_cb, callback_args=(n, suff))

    def subscribe_on_topics(self):
        return

    def create_publishers(self):
        return

    def _disarm_cb(self, msg):
        if msg.data:
            self.disarm = True

    def _control_cb(self, msg, callback_args):
        suff, = callback_args
        self._data[suff] = msg

    def _topic_cb(self, msg, callback_args):
        n, suff = callback_args
        self._data[n][suff] = msg

    def _service_proxy(self, n, path, arg_type, *args, **kwds):
        service = rospy.ServiceProxy(f"/mavros{n}/{path}", arg_type)
        ret = service(*args, **kwds)

    def _subscribe_on_copters_topics(self):
        self._subscribe_on_mavros_topics("local_position/pose", PoseStamped)
        self._subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)
        self._subscribe_on_mavros_topics("state", State)
        self._subscribe_on_mavros_topics("extended_state", ExtendedState)
        rospy.Subscriber(f"/{self.node_name}/disarm", Bool, self._disarm_cb)
        self.subscribe_on_topics()

    def _on_shutdown_cb(self):
        rospy.logfatal("shutdown")

    def _arming(self, n, to_arm):
        d = self._data[n].get("state")
        if d is not None and d.armed != to_arm:
            self._service_proxy(n, "cmd/arming", CommandBool, to_arm)

    def _set_mode(self, n, new_mode):
        d = self._data[n].get("state")
        if d is not None and d.mode != new_mode:
            self._service_proxy(n, "set_mode", SetMode, custom_mode=new_mode)

    def mc_takeoff(self, pt, n, dt):

        # TODO: REMOVE LINE BELOW
        self._arming(n, True)

        if dt < self.takeoff_timeout:
            self.set_vel(pt, 0, 0, 0.3)
            if dt > self.takeoff_timeout / 2:
                self._arming(n, True)

    def set_pos(self, pt, x, y, z):
        pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ
        pt.position.x = x
        pt.position.y = y
        pt.position.z = z

    def set_vel(self, pt, x, y, z):
        pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ
        pt.velocity.x = x
        pt.velocity.y = y
        pt.velocity.z = z

    def control_raw(self, pt, n, dt):
        self.mc_takeoff(pt, n, dt)
        walls_new = self._data.get("/path_generator/walls")
        central_new = self._data.get("/path_generator/central")
        copter_pose = self._data[n].get("local_position/pose")
        if walls_new is None or central_new is None or copter_pose is None:
            return
        walls_new = walls_new.data.split(' ')
        self.walls[walls_new[1]] = []
        for i in range(len(walls_new[2:]) // 4):
            self.walls[walls_new[1]].append([float(num) for num in walls_new[(2 + i * 4):(2 + i * 4) + 4]])
        central_new = central_new.data.split(' ')
        self.central[central_new[1]] = []
        for i in range(len(central_new[2:]) // 3):
            self.central[central_new[1]].append([float(num) for num in central_new[(2 + i * 3):(2 + i * 3) + 3]])
        copter_pose = copter_pose.pose.position
        self.copter_poses[n] = [copter_pose.x, copter_pose.y, copter_pose.z]
        self.control_user(pt, n, dt, self.walls, self.central, self.copter_poses)

    def control_user(self, pt, n, dt, walls, central, copter_pose):
        self.set_pos(pt, 0, 0, n)

    def _offboard_loop(self):
        pub_pt = {}

        for n in range(1, self.instances_num + 1):
            pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)

        pt = PositionTarget()
        pt.coordinate_frame = pt.FRAME_LOCAL_NED

        self.t0 = time()

        rate = rospy.Rate(self._freq)
        while not rospy.is_shutdown():
            if self.disarm:
                self.set_vel(pt, 0.0, 0.0, -1.0)
                for n in range(1, self.instances_num + 1):
                    pub_pt[n].publish(pt)
                break
            dt = time() - self.t0
            for n in range(1, self.instances_num + 1):
                self._set_mode(n, "OFFBOARD")
                self.control_raw(pt, n, dt)
                pub_pt[n].publish(pt)
            rate.sleep()

    def main(self):
        rospy.init_node(self.node_name)
        rospy.loginfo(f"{self.node_name} started with {self.instances_num} drones")

        self.create_publishers()
        self._subscribe_on_copters_topics()
        rospy.on_shutdown(self._on_shutdown_cb)

        try:
            self._offboard_loop()
        except rospy.ROSInterruptException:
            pass

        rospy.spin()


if __name__ == '__main__':
    controller = MultirotorController()
    controller.main()

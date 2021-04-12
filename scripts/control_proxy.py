#!/usr/bin/env python3
# coding=utf8

from controller import MultirotorController

import rospy
import time
import math

from std_msgs.msg import String, Int64, Bool
from geometry_msgs.msg import QuaternionStamped
from mavros_msgs.msg import PositionTarget


class ControlProxy(MultirotorController):
    def __init__(self, freq=100, node_name="offboard_node"):
        super().__init__(freq=freq, node_name=node_name)

    drones_num = 0
    drones_mode = ""

    target_x = 0.0
    target_y = 0.0
    target_z = 0.0

    drone_yaw = 0
    yaw_rate = 0

    MAX_LINEAR_PLANE = 12
    MAX_LINEAR_VERTICAL = 1
    MAX_ANG_VEL = -4
    POINT_VEL_K = 1.0

    data = {}
    lz = {}

    R = 2

    def control_pos_formation_vector(self, n):
        matrix = {
            1: [-2 * self.R, -2 * self.R, 0],
            2: [-2 * self.R, 2 * self.R, 0],
            3: [2 * self.R, -2 * self.R, 0],
            4: [2 * self.R, 2 * self.R, 0],
            5: [-self.R, -self.R, self.R],
            6: [-self.R, self.R, self.R],
            7: [self.R, -self.R, self.R],
            8: [self.R, self.R, self.R],
            9: [-self.R, -self.R, 2 * self.R],
            10: [-self.R, self.R, 2 * self.R],
            11: [self.R, -self.R, 2 * self.R],
            12: [self.R, self.R, 2 * self.R],
            13: [-2 * self.R, -2 * self.R, 3 * self.R],
            14: [-2 * self.R, 2 * self.R, 3 * self.R],
            15: [2 * self.R, -self.R, 3 * self.R],
            16: [2 * self.R, 2 * self.R, 3 * self.R],
        }
        if n not in matrix.keys():
            return [0, 0, 0]
        return matrix.get(n)

    def subscribe_on_control_topics(self):
        rospy.Subscriber("/joy/control", QuaternionStamped, self._control_cb, callback_args=("/joy/control",))
        rospy.Subscriber("/joy/control/mode", String, self._control_cb, callback_args=("/joy/control/mode",))
        rospy.Subscriber("/joy/control/drones", Int64, self._control_cb, callback_args=("/joy/control/drones",))
        rospy.Subscriber("/joy/control/takeoff", Bool, self._control_cb, callback_args=("/joy/control/takeoff",))

    def set_pos(self, pt, x, y, z):
        pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | \
                       pt.IGNORE_YAW_RATE
        pt.position.x = x
        pt.position.y = y
        pt.position.z = z
        pt.yaw = self.drone_yaw

    def set_vel(self, pt, x, y, z):
        pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | \
                       pt.IGNORE_YAW_RATE
        pt.velocity.x = x
        pt.velocity.y = y
        pt.velocity.z = z
        pt.yaw = self.drone_yaw

    def control_raw(self, pt, n, dt):
        if self._data.get('/joy/control') is None \
                or self._data.get('/joy/control/mode') is None \
                or self._data.get('/joy/control/drones') is None \
                or self._data.get('/joy/control/takeoff') is None:
            return
        if self._data.get('/joy/control/takeoff').data:
            rospy.logwarn(f"TAKEOFF")
            self.t0 = time.time()
            self._data.get('/joy/control/takeoff').data = False
        self.set_vel(pt, 0, 0, 0)
        self.mc_takeoff(pt, n, dt)
        if n <= self.drones_num:
            if self._data.get('/joy/control/mode').data == "global vel":
                self.yaw_rate = self._data.get('/joy/control').quaternion.w * self.MAX_ANG_VEL
                self.set_vel(pt, self._data.get('/joy/control').quaternion.x * self.MAX_LINEAR_PLANE,
                             self._data.get('/joy/control').quaternion.y * self.MAX_LINEAR_PLANE,
                             self._data.get('/joy/control').quaternion.z * self.MAX_LINEAR_VERTICAL)
            if self._data.get('/joy/control/mode').data == "local vel":
                self.yaw_rate = self._data.get('/joy/control').quaternion.w * self.MAX_ANG_VEL
                teta = self.drone_yaw + self.yaw_rate / self._freq - math.pi / 2
                global_x = self._data.get('/joy/control').quaternion.x * self.MAX_LINEAR_PLANE
                global_y = self._data.get('/joy/control').quaternion.y * self.MAX_LINEAR_PLANE
                local_x = global_x * math.cos(teta) - global_y * math.sin(teta)
                local_y = global_x * math.sin(teta) + global_y * math.cos(teta)
                self.set_vel(pt, local_x, local_y,
                             self._data.get('/joy/control').quaternion.z * self.MAX_LINEAR_VERTICAL)
            if self._data.get('/joy/control/mode').data == "global pos":
                teta = self.drone_yaw + self.yaw_rate / self._freq - math.pi / 2
                self.yaw_rate = self._data.get('/joy/control').quaternion.w * self.MAX_ANG_VEL
                dx, dy, dz = self.control_pos_formation_vector(n)
                local_dx = dx * math.cos(teta) - dy * math.sin(teta)
                local_dy = dx * math.sin(teta) + dy * math.cos(teta)
                self.target_x += self._data.get(
                    '/joy/control').quaternion.x * self.MAX_LINEAR_PLANE / self._freq * self.POINT_VEL_K
                self.target_y += self._data.get(
                    '/joy/control').quaternion.y * self.MAX_LINEAR_PLANE / self._freq * self.POINT_VEL_K
                self.target_z += self._data.get(
                    '/joy/control').quaternion.z * self.MAX_LINEAR_VERTICAL / self._freq * self.POINT_VEL_K
                self.set_pos(pt, self.target_x + local_dx, self.target_y + local_dy, self.target_z + dz)
            if self._data.get('/joy/control/mode').data == "local pos":
                teta = self.drone_yaw + self.yaw_rate / self._freq - math.pi / 2
                self.yaw_rate = self._data.get('/joy/control').quaternion.w * self.MAX_ANG_VEL * self.POINT_VEL_K
                dx, dy, dz = self.control_pos_formation_vector(n)
                local_dx = dx * math.cos(teta) - dy * math.sin(teta)
                local_dy = dx * math.sin(teta) + dy * math.cos(teta)
                global_x = self._data.get('/joy/control').quaternion.x * self.MAX_LINEAR_PLANE
                global_y = self._data.get('/joy/control').quaternion.y * self.MAX_LINEAR_PLANE
                self.target_x += (global_x * math.cos(teta) - global_y * math.sin(teta)) / self._freq * self.POINT_VEL_K
                self.target_y += (global_x * math.sin(teta) + global_y * math.cos(teta)) / self._freq * self.POINT_VEL_K
                self.target_z += self._data.get(
                    '/joy/control').quaternion.z * self.MAX_LINEAR_VERTICAL / self._freq * self.POINT_VEL_K
                self.set_pos(pt, self.target_x + local_dx, self.target_y + local_dy, self.target_z + dz)

    def _offboard_loop(self):

        pub_pt = {}

        for n in range(1, self.instances_num + 1):
            pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)

        pt = PositionTarget()

        pt.coordinate_frame = pt.FRAME_LOCAL_NED

        self.t0 = time.time()

        rate = rospy.Rate(self._freq)
        while not rospy.is_shutdown():
            if self._data.get('/joy/control/drones') is None:
                rate.sleep()
                continue
            if self._data.get('/joy/control/drones').data > self.instances_num:
                rospy.logerr(
                    f"Can't control: {self._data.get('/joy/control/drones').data} drones, "
                    f"limit is {self.instances_num}")
                self._data.get('/joy/control/drones').data = self.instances_num
            if self._data.get('/joy/control/drones').data != self.drones_num:
                self.drones_num = self._data.get('/joy/control/drones').data
                rospy.logwarn(f"Drones controlled: {self.drones_num}")
            if self._data.get('/joy/control/mode').data != self.drones_mode:
                self.drones_mode = self._data.get('/joy/control/mode').data
                rospy.logwarn(f"Control mode: {self.drones_mode}")

                self.drone_yaw += self.yaw_rate * 1 / self._freq
            self.drone_yaw = self.drone_yaw % (2 * math.pi)
            dt = time.time() - self.t0

            for n in range(1, self.drones_num + 1):
                self._set_mode(n, "OFFBOARD")
                self.control_raw(pt, n, dt)
                pub_pt[n].publish(pt)

            rate.sleep()


controller = ControlProxy()
controller.main()

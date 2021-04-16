from random import random
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform
from time import sleep

pub_target = rospy.Publisher("/physx_path_planner/target_vectors", MultiDOFJointTrajectory, queue_size=1)

X = 3
Y = 3
Z = 3
R = 5.0
L = 1
height = 10

while True:
    first = []
    second = []

    for x in range(X):
        for y in range(Y):
            for z in range(Z):
                first.append([R * (random() - 0.5) + x * L - X // 2 * L,
                              R * (random() - 0.5) + y * L - Y // 2 * L,
                              R * (random() - 0.5) + z * L - Z // 2 * L])
                second.append([x * L - X // 2 * L,
                               y * L - Y // 2 * L,
                               z * L - Z // 2 * L + height])

    points = MultiDOFJointTrajectory()
    points.header.frame_id = "-1"
    point_first = MultiDOFJointTrajectoryPoint()
    point_second = MultiDOFJointTrajectoryPoint()

    for point in first:
        transform = Transform()
        transform.translation.x = point[0]
        transform.translation.y = point[1]
        transform.translation.z = point[2]
        point_first.transforms.append(transform)

    for point in second:
        transform = Transform()
        transform.translation.x = point[0]
        transform.translation.y = point[1]
        transform.translation.z = point[2]
        point_second.transforms.append(transform)

    points.points.append(point_first)
    points.points.append(point_second)

    rospy.init_node("random_target_generator")
    pub_target.publish(points)

    sleep(0.1)

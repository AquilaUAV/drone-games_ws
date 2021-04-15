

from pyphysx import *
from pyphysx_utils.rate import Rate
from pyphysx_render.pyrender import PyPhysxViewer
from pyphysx_render.meshcat_render import MeshcatViewer
from time import time
from random import randint, random
import numpy as np

Physics.init_gpu()

scene = Scene(scene_flags=[ENABLE_GPU_DYNAMICS, ], )

# scene.add_actor(RigidStatic.create_plane(material=Material(static_friction=0.0, dynamic_friction=0.0, restitution=0.0)))

L = 4

for i in range(10):
    actor = RigidDynamic()
    shape = Shape.create_sphere(1.2, Material(restitution=0.0, dynamic_friction=0.5, static_friction=0.0))
    # shape.set_flag(SIMULATION_SHAPE, False)
    actor.attach_shape(shape)
    actor.set_global_pose([(random() - 0.5) * L, (random() - 0.5) * L, random() * 1])
    actor.disable_gravity()
    actor.set_mass(1.0)
    actor.set_max_angular_velocity(0)
    actor.set_max_linear_velocity(12)
    scene.add_actor(actor)

# render = PyPhysxViewer(wait_for_open=True)  # video_filename='videos/01_free_fall.gif'
render = MeshcatViewer(wait_for_open=True, open_meshcat=True)
render.clear_physx_scenes()
render.add_physx_scene(scene)

last_time = time()
counter = 0

rate = Rate(60)
while render.is_active:
    # rate.sleep()
    counter += 1
    # watch.set_global_pose()
    # print(watch.get_global_pose())
    if time() - last_time > 1.0:
        actor = RigidDynamic()
        shape = Shape.create_sphere(1.2, Material(restitution=0.0, dynamic_friction=0.5, static_friction=0.0))
        # shape.set_flag(SIMULATION_SHAPE, False)
        actor.attach_shape(shape)
        actor.set_global_pose([(random() - 0.5) * L, (random() - 0.5) * L, random() * 1])
        actor.disable_gravity()
        actor.set_mass(1.0)
        actor.set_max_angular_velocity(0)
        actor.set_max_linear_velocity(12)
        scene.add_actor(actor)
        render.clear_physx_scenes()
        render.add_physx_scene(scene)
        print(counter / 1)
        counter = 0
        last_time = time()
    print(len(scene.get_dynamic_rigid_actors()))
    for actor in scene.get_dynamic_rigid_actors():
        pose = actor.get_global_pose()[0]
        actor.set_global_pose([pose[0], pose[1], 0.0])
        pose[0] = -pose[0]
        pose[1] = -pose[1]
        pose[2] = 0.0
        actor.set_linear_velocity(vel=pose)

    scene.simulate(rate.period())
    render.update()

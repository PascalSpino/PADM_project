from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import math
from random import random
from collections import namedtuple
from itertools import product, combinations

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet', 'src'])


from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import remove_body, get_joint_positions, get_bodies, get_body_name, quat_from_euler, create_attachment, plan_cartesian_motion, wait_for_duration
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, is_circular, get_joint_type, get_joints, get_joint_info, get_collision_data, get_joint_ancestors, are_links_adjacent, circular_difference, wrap_positions

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
# from world import open_door
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

UNIT_POSE2D = (0., 0., 0.)

##############################################################################
# RRT

class rrtNode(object):

    def __init__(self, qs, parent):
        self._parent = parent
        self._q = qs

    def path(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node._parent
        return sequence[::-1]

def map_path(nodes):
    return list(map(lambda n: n._q, nodes))

def rrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, max_iterations=100, goal_probability=.5):
    if collision_fn(start):
        # start condition is in collision, no solution
        return None
    nodes = [rrtNode(start, None)]

    for i in range(max_iterations):
        # decide rather to sample from goal or randomly
        r = random()
        if i < 5 or r <= goal_probability:
            sample = goal
        else:
            sample = sample_fn()
        # find nearest existing node on rrt graph
        last = nodes[np.argmin([distance_fn(n._q, sample) for n in nodes])]
        # check for collision (and limit extension to unit distance)
        for q in extend_fn(last._q, sample):
            if collision_fn(q):
                break
            # add node to rrt graph
            last = rrtNode(q, last)
            nodes.append(last)
            # check for goal condition
            if last._q == goal:
                # goal achieved
                return map_path(last.path())
    return None

###############################################################################
# Collision, Extend, Distance, Sample, Difference and Refine functions for RRT

def get_refine_fn(body, joints, num_steps):
    # limits change between two joint configurations based on c-space euclidean distance
    # necessary for rrt
    difference_fn = get_difference_fn(body, joints)
    num_steps = num_steps + 1
    def fn(q1, q2):
        q = q1
        for i in range(num_steps):
            positions = (1.0 / (num_steps - i)) * np.array(difference_fn(q2, q)) + q
            q = tuple(wrap_positions(body, joints, positions))
            yield q
    return fn

def get_difference_fn(body, joints):
    # defines vector distance between two joint configurations vectors
    def fn(q2, q1):
        return tuple((value2 - value1) for value2, value1 in zip(q2, q1))
    return fn

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def get_distance_fn(body, joints):
    def fn(q1, q2):
        # distance between configurations is the root sum of square differences between each joint, pairwise
        # aka euclidean distance in C-space
        return np.sqrt(sum([(q2[i] - q1[i])**2 for i in range(len(q1))]))
    return fn

def get_extend_fn(body, joints):
    # defines a function that increments between two joint configurations (with refine)
    res = np.array([0.05 for joint in joints])
    def fn(q1, q2):
        steps = int(np.linalg.norm(np.divide([(q2[i] - q1[i]) for i in range(len(q1))], res), ord=2))
        refine_fn = get_refine_fn(body, joints, steps)
        return refine_fn(q1, q2)
    return fn

def get_collision_fn(body, joints, obstacles=[]):
    # checks for collision between body and bodies in the obstacles list
    def fn(q):
        set_joint_positions(body, joints, q)
        for obs in obstacles:
        # if world.robot != other_body and pairwise_collision(world.robot, other_body):
            if pairwise_collision(body, obs):
                return True
        return False
    return fn

###############################################################################
# Autonomy Functions

def drive_to_counter(world):
    # Driving base to goal position
    start_pose = get_joint_positions(world.robot, world.base_joints)
    end_pos = [0.8, 0.75]
    heading = np.arctan2(end_pos[1] - start_pose[1], end_pos[0] - start_pose[0])
    for rot in interpolate_rot(np.pi, heading, rot_step_size=0.0001):
        curr_pose = (start_pose[0], start_pose[1], rot)
        set_joint_positions(world.robot, world.base_joints, curr_pose)

    for pos in interpolate_pos((start_pose[0], start_pose[1]), end_pos, pos_step_size=0.0001):
        curr_pose = (pos[0], pos[1], heading)
        set_joint_positions(world.robot, world.base_joints, curr_pose)
    set_joint_positions(world.robot, world.base_joints, (end_pos[0], end_pos[1], np.pi))

def pick_up(world, obj, obj_pose):
    # collecting objects
    tool_link = link_from_name(world.robot, 'panda_hand')
    joints = get_movable_joints(world.robot)
    first_joint= world.arm_joints[0]

    # collecting RRT functions
    obstacles = []
    for b in get_bodies():
        if world.robot != b:
            obstacles.append(b)
    distance_fn = get_distance_fn(world.robot, world.arm_joints)
    sample_fn = get_sample_fn(world.robot, world.arm_joints)
    extend_fn = get_extend_fn(world.robot, world.arm_joints)
    collision_fn = get_collision_fn(world.robot, world.arm_joints, obstacles)

    # get start configuration and grasp pose
    start = (get_joint_positions(world.robot, joints))[9:16]
    [end_pos, end_rot] = obj_pose
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pos[2] =  end_pos[2] + 0.2
    target_pose = [end_pos, end_rot]
    # convert grasp pose to c-space configuration goal
    full_conf = plan_cartesian_motion(world.robot, first_joint, tool_link, [target_pose])
    goal = full_conf[0][9:16]
    # resent initial configuration
    set_joint_positions(world.robot, world.arm_joints, start)
    # perform RRT
    confs = rrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, goal_probability=0.2)
    # execute RRT graph
    for conf in confs:
        set_joint_positions(world.robot, world.arm_joints, conf)
        wait_for_duration(1e-2)

############################################################################
# Simulation Functions

def interpolate_pos(pos1, pos2, pos_step_size=0.01):
    sq_sum = 0.0
    for i in range(len(pos1)):
        sq_sum += (pos1[i] - pos2[i])**2
    dist = np.sqrt(sq_sum)
    num_steps = int(math.ceil(dist/pos_step_size))
    for i in range(num_steps):
        fraction = float(i) / num_steps
        pos = (1-fraction)*np.array(pos1) + fraction*np.array(pos2)
        pos = (pos1[0] + (pos2[0] - pos1[0]) * fraction, pos1[1] + (pos2[1] - pos1[1]) * fraction)
        yield pos
    yield pos2

def interpolate_rot(rot1, rot2, rot_step_size=0.01):
    num_step = int(math.ceil(abs(rot2-rot1)/rot_step_size))
    for i in range(num_step):
        yield float(i) / num_step * rot_step_size + rot1
    yield rot2

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose3D = pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name, pose3D

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

###########################################################################

def main():
    # performing setup
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())
    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box, sugar_box_pose = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box, spam_box_pose = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    remove_body(world.gripper)
    world._update_initial()

    wait_for_user()

    drive_to_counter(world)

    wait_for_user()

    pick_up(world, spam_box, spam_box_pose)

    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()

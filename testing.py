from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import math

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet', 'src'])


from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import remove_body, get_joint_positions, get_bodies, get_body_name, quat_from_euler, create_attachment
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
# from world import open_door
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

UNIT_POSE2D = (0., 0., 0.)

def goto(ik_joints, world, tool_link, end_pose):
    start_pose = get_link_pose(world.robot, tool_link)
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                print('Failure!')
                wait_for_user()
                break
            set_joint_positions(world.robot, ik_joints, conf)


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


def print_collisions():
    for body1 in get_bodies():
        for body2 in get_bodies():
            if pairwise_collision(body1, body2):
                print(f"{body1} ({get_body_name(body1)}) collides with {body2} ({get_body_name(body2)})")

def robot_collision(world):
    for other_body in get_bodies():
        # if world.robot != other_body and pairwise_collision(world.robot, other_body):
        if pairwise_collision(world.robot, other_body):
            print(f"robot collides with {other_body} ({get_body_name(other_body)})")
            return True
    return False

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

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def main():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box, sugar_box_pose = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box, spam_box_pose = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    remove_body(world.gripper)
    wait_for_user()
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    joints = get_movable_joints(world.robot)
    sample_fn = get_sample_fn(world.robot, world.arm_joints)

    for body in get_bodies():
        print(f"BODIES: {body} ({get_body_name(body)})")

    wait_for_user()

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

    # Setting up arm IK
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)

    wait_for_user()
    
    print("opening drawer\n")
    start_pose = get_link_pose(world.robot, tool_link)
    end_pos = [0.3, 1.1, -0.6]
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pose = [end_pos, end_rot]
    goto(ik_joints, world, tool_link, end_pose)

    end_pos = [0.4, 1.1, -0.6]
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pose = [end_pos, end_rot]
    goto(ik_joints, world, tool_link, end_pose)
    world.open_door(56)
    # wait_for_user()

    print("reaching for spam\n")
    start_pose = get_link_pose(world.robot, tool_link)

    [end_pos, end_rot] = spam_box_pose
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pos[2] = -0.3
    end_pose = [end_pos, end_rot]
    goto(ik_joints, world, tool_link, end_pose)
    end_pos[2] = -0.4
    end_pose = [end_pos, end_rot]
    goto(ik_joints, world, tool_link, end_pose)

    # wait_for_user()

    start_pose = get_link_pose(world.robot, tool_link)
    end_pos = [0.4, 1.1, -0.4]
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pose = [end_pos, end_rot]
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            set_pose(5, pose)
            if conf is None:
                print('Failure!')
                wait_for_user()
                break
            set_joint_positions(world.robot, ik_joints, conf)
    start_pose = get_link_pose(world.robot, tool_link)
    end_pos = [0.4, 1.1, -0.6]
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pose = [end_pos, end_rot]
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            set_pose(5, pose)
            if conf is None:
                print('Failure!')
                wait_for_user()
                break
            set_joint_positions(world.robot, ik_joints, conf)

    wait_for_user()

    

    
    # print("Going to use IK to go from a sample start state to a goal state\n")
    # for i in range(5):
    #     print('Iteration:', i)
    #     conf = sample_fn()

    #     # start_conf = (0,np.pi/2,0,0,0,np.pi,0)
    #     set_joint_positions(world.robot, world.arm_joints, conf)
    #     wait_for_user()

    #     ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    #     start_pose = get_link_pose(world.robot, tool_link)

    #     [end_pos, end_rot] = spam_box_pose
    #     end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    #     end_pos[2] = -0.4
    #     end_pose = [end_pos, end_rot]
    #     # end_pose[0][2] = -0.2
    #     # end_pose[1] = end_rot
    #     print("goal pose: ", end_pose)
    #     # end_pose = multiply(Pose(Point(x=1,y=1,z=0.01)))
    #     for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
    #         conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
    #         if conf is None:
    #             print('Failure!')
    #             wait_for_user()
    #             break
    #         set_joint_positions(world.robot, ik_joints, conf)
    #     print('pose: ', get_link_pose(world.robot, tool_link))
    #     wait_for_user()


    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()

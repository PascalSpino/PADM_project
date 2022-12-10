from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import math
from time import *

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet', 'src'])


from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import remove_body, get_joint_positions, get_bodies, get_body_name, quat_from_euler, create_attachment
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, set_joint_position, interval_generator, get_link_pose, interpolate_poses

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
    i = 1
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
        conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
        if conf is None:
            print('Failure!')
            wait_for_user()
            break
        # Time delay to mitigate appearance of teleportation
        sleep(0.01)
        i += 1
        set_joint_positions(world.robot, ik_joints, conf)

def goto_open_drawer(ik_joints, world, tool_link, end_pose):

    # X-axis minimum value for closed drawer
    drawer_x_min = -0.008500000461935997
    # X-axis maximum value for open drawer
    drawer_x_max = 0.2915000021457672
    # offset for set_joint_position to line up exactly with 3-d position
    offset = 0.0005
    # Opening happens across 36 poses
    shift_amount = (drawer_x_max - drawer_x_min) / 36

    start_pose = get_link_pose(world.robot, tool_link)

    i = 1
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
        conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
        if conf is None:
            print('Failure!')
            #wait_for_user()
            #break
            continue
        # Time delay to mitigate appearance of teleportation
        sleep(0.01)
        i += 1
        set_joint_positions(world.robot, ik_joints, conf)
        set_joint_position(world.kitchen, 56, offset + drawer_x_min + i * shift_amount)

def goto_close_drawer(ik_joints, world, tool_link, end_pose):
    # X-axis minimum value for closed drawer
    drawer_x_min = -0.008500000461935997
    # X-axis maximum value for open drawer
    drawer_x_max = 0.2915000021457672
    # offset for set_joint_position to line up exactly with 3-d position
    offset = 0.0005
    # Opening happens across 36 poses
    shift_amount = (drawer_x_max - drawer_x_min) / 36

    start_pose = get_link_pose(world.robot, tool_link)

    i = 1
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
        conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
        # Offset spam spam position from gripper while syncing their movements
        spam_can_pose  = pose
        spam_can_pose[0][0] -= 0.5
        set_pose(5, spam_can_pose)
        
        if conf is None:
            print('Failure!')
            #wait_for_user()
            #break
            continue
        # Time delay to mitigate appearance of teleportation
        sleep(0.01)
        i += 1
        set_joint_positions(world.robot, ik_joints, conf)
        set_joint_position(world.kitchen, 56, offset + drawer_x_max - i * shift_amount)



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

    # Drive to counter
    drive_to_counter(world)

    # Setting up arm IK
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)


    wait_for_user()

    #print("Move to sugar box\n")
    #start_pose = get_link_pose(world.robot, tool_link)

    #[end_pos, end_rot] = sugar_box_pose
    #end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    #end_pos[2] = -0.3
    #end_pose = [end_pos, end_rot]
    #goto(ik_joints, world, tool_link, end_pose)
    #print(end_pos)
    #end_pos[2] = -0.4
    #end_pose = [end_pos, end_rot]
    #goto(ik_joints, world, tool_link, end_pose)

    #wait_for_user()

   



    print("\n[+] Rotating gripper to grip drawer handle\n")
    # Position gripper to open drawer
    start_pose = get_link_pose(world.robot, tool_link)
    end_pos = [0.3, 1.1, -0.6] # positioning gripper to open drawer
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pose = [end_pos, end_rot]
    goto(ik_joints, world, tool_link, end_pose)
    
    print("\n[+] Syncing gripper with opening drawer\n")
    wait_for_user()
    # Gripper movement during drawer opening
    end_pos = [0.6, 1.1, -0.6] # end_pos of gripper
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pose = [end_pos, end_rot]

    # Move gripper and open drawer
    goto_open_drawer(ik_joints, world, tool_link, end_pose)
    

    print("\n[+] Drawer opened\n")
    wait_for_user()

    print("\n[+] Position gripper above door handle\n")
    
    # Positioning gripper above door handle
    end_pos = [0.6, 1.1, -0.2] # end_pos of gripper
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pose = [end_pos, end_rot]
    goto(ik_joints, world, tool_link, end_pose)
    wait_for_user()

    print("\n[+] Picking up spam can\n")
    start_pose = get_link_pose(world.robot, tool_link)

    [end_pos, end_rot] = spam_box_pose
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pos[2] = -0.3
    end_pose = [end_pos, end_rot]
    goto(ik_joints, world, tool_link, end_pose)
    end_pos[2] = -0.4
    end_pose = [end_pos, end_rot]
    goto(ik_joints, world, tool_link, end_pose)

    wait_for_user()
    
    print("\n[+] Position spam can over drawer\n")
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
    
    wait_for_user()
    
    print("\n[+] Lowering spam can into drawer\n")
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

    print("\n[+] Positioning gripper to close drawer\n")
    
    # Position gripper above door handle
    end_pos = [0.6, 1.1, -0.2] # end_pos of gripper
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pose = [end_pos, end_rot]
    goto(ik_joints, world, tool_link, end_pose)
    
    wait_for_user()

    # Position gripper to close drawer
    end_pos = [0.6, 1.1, -0.6] # end_pos of gripper
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pose = [end_pos, end_rot]
    goto(ik_joints, world, tool_link, end_pose)
    
    wait_for_user()


    print("\n[+] Closing drawer\n")

    # Sync gripper motion with door closing motion
    start_pose = get_link_pose(world.robot, tool_link)
    end_pos = [0.3, 1.1, -0.6] # positioning gripper to open drawer
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pose = [end_pos, end_rot]
    
    # Move gripper and close drawer
    goto_close_drawer(ik_joints, world, tool_link, end_pose)
    wait_for_user()

    # Retract gripper from drawer, tasks complete
    end_pos = [0.6, 1.1, -0.2] # end_pos of gripper
    end_rot = quat_from_euler([np.pi, 0.0, 0.0])
    end_pose = [end_pos, end_rot]
    goto(ik_joints, world, tool_link, end_pose)

    wait_for_user()

    print("\n[+] Hit enter to call world.destroy()\n")
    wait_for_user()
    world.destroy()

if __name__ == '__main__':
    main()

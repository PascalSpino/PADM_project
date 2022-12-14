# PADM Final Project

## Introduction 

This project incorporates three major topics covered in class: activity planning, motion planning, and trajectory optimization. The high-level task was to simulate a pick-and-place robot problem. We were given a Franka robot mounted to a carter spawned into a kitchen environment. The kitchen contained a stove with a sugar box placed on a burner, a spam can placed on a counter, as well as several drawers and cabinets. We had two objectives, to be completed in either order

* Navigate to the sugar box. Pick it up off the stove and place in anywhere on a countertop.
* Navigate to the spam can. Stow the spam can in the indigo drawer.

Below is documentation of our approach. This repo contains all products produced for the Final Project of Principles of Autonomy and Decision Making (6.4132/16.412, Fall '22).

## Activity Planning 

### Implementation

**1. Qualitatively mention your assumptions that you made while designing the domain**
- The ‘spam can’ and ‘sugar box’ objects can only exist in one of four locations: the burner, the countertop, the drawer, or within the gripper.
- The gripper can only hold one object at a time.
- The gripper does not need to rotate to open the drawer vs. pick up objects.
- In order to achieve the goal state, the only necessary conditions are that the spam can is in the drawer, the sugar box is on the counter, and the drawer is closed.
- The robot must drive closer to the counter in order to manipulate the objects, but is capable of reaching all objects from one location once it is close enough.
- The robot has complete knowledge of its surroundings and does not need to perform any search actions to find the objects.
- The robot acts deterministically and does not need to recover from events such as dropping an object, etc… 
- At this stage, the duration of actions do not need to be accounted for in the planning domain. 

**2. Explain the files and the approach you used to generate the plan**

We followed the documentation available on Planning.wiki (https://planning.wiki/ref/pddl/problem) to generate our own domain and problem PDDL files. The documentation covered a different problem, but one that was adaptable to this project. Our approach was to develop the simplest possible set of predicates and actions for path planning specific to this problem, rather than develop predicates and actions for a more general case. 

- The `domain.pddl` file contains our final domain specifications. We had four predicates that can be applied to objects, including `on_counter ?x`, `in_drawer ?x`, `on_burner ?x`, and `gripper_holding ?x`. We had two states to represent the drawer (`drawer_open` and `drawer_closed`), two to represent the robot's location relative to the counter (`robot_far_from_counter` and `robot_close_to_counter`), and one to represent the state of the gripper (`gripper_empty`). For operations with these predicates, we defined the following actions: `pick_up`, `place_in_drawer`, `place_on_counter`, `open_drawer`, `close_drawer`, and `drive_to_counter`. By design, only the first three of these actions requre an object to be provided. We chose to omit using the drawer as an object, and thus the open/close drawer actions did not require provision of an object.
- The `problem.pddl` file contains our problem in PDDL format. We defined the problem as having only two objects, `spam` and `sugar`. We then defined the initial state of the problem as specified in the project specifications, with the following predicates applied: `on_burner sugar`, `on_counter spam`, `gripper_empty`, `drawer_closed`, and `robot_far_from_counter`. These were sufficient to characterize the initial state. We also defined the goal in the problem file, containing the requirements: `on_counter sugar`, `in_drawer spam`, and `drawer_closed`.
- The `planner.py` file contains our activity planner implentation. We used the suggested `pddl-parser` library and a `Breadth First Search` approach. As per expectations, we made our own BFS implentation using Node objects to represent each state. Each Node object contains an instance variable that indicate's its parent (or None for the starting state). After a state containing the goal requirements is found, we generate the path by iterating backwards through the parent Nodes to determine the order taken from the start state to the goal.

**3. You can also mention any challenges you faced and things that you tried irrespective of whether they worked or not**

- There are several versions of PDDL and the syntax varies slightly between them. We focused primarily on PDDL 2.1 but suspected that we would later have to make some modifications in order for our .pddl files to be parsed successfully. At first, we tried to craft our predicates with objects for the drawer, countertop, gripper, spam can, sugar box, etc. Ultimately, we instead reduced our set of objects to just the spam can and sugar box, and crafted predicates to describe the drawer state and object locations without additional objects. 
- Choosing to orient our predicates around the spam can and sugar box objects led us to heavily use negations of predicates in both our `domain.pddl` and `problem.pddl` files. This would have been fine theoretically, but having many negations sometimes made it more diffucult to debug the logic before we got our activity planner working. Ultimately we made a handful of changes, including only having positive initial predicates, only having positive goals, and including a couple pairs of predicates that represented opposite states for specific items (i.e. `drawer_open` and `drawer_closed`, as well as `robot_close_to_counter` and `robot_far_from_counter`). Using a couple paired predicates made our `.pddl` files slightly less consise, but a bit easier to debug and understand.

### Results

Our activity planner can be run from the command-line in the same manner as in the [pddl-parser](https://github.com/pucrs-automated-planning/pddl-parser) documentation, albeit slightly simplified:

```Shell
(.venv) ubuntu@ubuntu:~/$ python -B -m planner domain.pddl problem.pddl
```

We used a BFS approach. The order of the steps to complete are dependent on the order in which actions are added to the queue. Rearranging the action order in `domain.pddl` can yield the alternate sequence of actions to complete both tasks. For our motion planning, we used the ordering in which the spam can task is completed first before moving the sugar box.

<details open><summary>Planner output</summary>

```Shell
[+] Found plan after 66 node expansions.
[+] Action sequence to complete both tasks:

Step 1: drive_to_counter 
Step 2: open_drawer 
Step 3: pick_up spam
Step 4: place_in_drawer spam
Step 5: close_drawer 
Step 6: pick_up sugar
Step 7: place_on_counter sugar

[+] As a list:
['drive_to_counter ', 'open_drawer ', 'pick_up spam', 'place_in_drawer spam', 'close_drawer ', 'pick_up sugar', 'place_on_counter sugar']
```
</details>

## Motion Planning

**1. Qualitatively mention your assumptions that you made about the about the environment, state space, and start and goal positions**
- The environment is static except for the movement of the arm itself and all objects within the environemt are initialized deterministically with consistent poses.
- The movement of the wheeled robot base is not necessary to performing manipulation tasks of the `drawer`, `spam` and `sugar` objects. The wheeled robot base also ideally operates in obstacle-free space, and so it does not make sense to encorperate RRT in this movement. In order to perform the 'drive_to_counter' action, the robot base executes a predefined series of motions to bring the robot arm within suitable range of the objects it must manipulate.  
- Goal states for the `spam` and `sugar` locations are somewhat arbitrary as long as they fulfill the following requirements: the `spam` object should be within the bounds of the `drawer` and not in collision with the kitchen, the `sugar` object should be on the counter relatively close to the original `spam` position, with no specific orientation.
- The arm is capable of perfectly grasping each object and must only get relatively close to the object (within the span of the gripper fingers) to do so. While the arm is holding an object, the object remains perfectly rigid with respect to the frame of the gipper.
- Modification of the `drawer` motion (as well as the attached `spam` when necessary) is hardcoded to move at speeds relative to the gripper motion. In addition, the goal base location for the `drive_to_counter` task and for object grasps are to some extent hardcoded. The visualization of all object motion is is accomplished by our code through subsequent `set_pose` calls (no collision/contact physics is performed by the simulator itself). 


**2. Explain the files and the motion planners you implemented**
- RRT was implemented in the configuration space of the 6-joint manipulator arm in order to plan each trajectory. The code for our implementation as well as definitions for each action from the planner can be found in `full_task_demo_polished.py`. The implementation of RRT is typical and builds upon the components `get_sample_fn`, `get_collision_fn`, `get_distance_fn` and `get_extend_fn` all designed to work in the configuration space of the arm. The RRT function itself, `def rrt(...)`, accepts as inputs each function collected from the previous definitions, as well as a `start` configuration, a `goal` configuration and assignments for `max_iterations` and `goal_probability`. The function first checks that `start` and `goal` do not produce collisions, and then proceeds to either randomly sample or sample from the goal based on `goal_probability`. For each sample, the nearest prior configuration on the RRT graph is calculated by joint euclidean distance and a new configuration is calculated by extending a limited distance towards the sample. With 0.05 radian resolution, each interpolated configuration towards this new configuration is tested for collision by calling pybullets `pairwise_collision` between the robot body and each other object in the world. If no collisions are found, this new configuration is added to the RRT graph and the process continues until either `max_iterations` is met or the goal configration is added to the RRT graph. 
- For trajectory planning with RRT while the arm is holding an object, a new collision-checking method is defined by `get_collision_holding_fn` that takes as additional input the held object and rigid body transform between the gripper and object. In addition to checking pairwise collision between the robot arm and all other objects in the world, the function additionally computes a new pose for the held object by applying forward kinematics to the arm kinematics, and multiplying this transform by the gripper-object transform. The new pose is also checked for collision to assure that RRT does not return a path where the held object collides with other objects. 
- For each task definition, goal configurations are calculated offline through a seperate pipeline we implemented to find feasible grasp positions. This pipeline allowed us to move the arm to positions that looked visually correct, and then extract a configuration for the planner.
- For the `open_drawer` and `close_drawer` tasks, due to the mechanical constraints of the expected drawer motion, we did not rely fully on RRT (which would have produced non-straight trajectories that would be physically inconsistent with the drawer motion). Instead, RRT is used to find a valid trajectory to the drawer grasp location, and then inverse kinematics is used to drive the manipulator along a straight path in the same direction that the drawer moves, at the same rate at which the drawer is moved. 

**3. Explain how you integrated the activity plan with the motion plan**
- We added an instance variable to the Planner class in `planner.py` so that the plan could be directly accessible without the need to read from a file. With this approach we chose to represent the plan as an ordered list where each element is a string containing the next step in the plan. After the `solve()` method is run, the instance variable `plan` of a `Planner` object will contain the BFS-generated plan: `['drive_to_counter ', 'open_drawer ', 'pick_up spam', 'place_in_drawer spam', 'close_drawer ', 'pick_up sugar', 'place_on_counter sugar']`

**4. GIF/video of the robot executing the plan**

[![](http://img.youtube.com/vi/Egf629Q4Xzc/0.jpg)](http://www.youtube.com/watch?v=Egf629Q4Xzc "Video Title")

[Click on video or link to watch demonstration](https://youtu.be/Egf629Q4Xzc)

<details open><summary>Terminal output</summary>

```Shell
(.venv) ubuntu@ubuntu:~/$ python full_task_demo_polished.py 
pybullet build time: May 20 2022 19:44:17
Random seed: 2147483648
Numpy seed: 2147483648
Loading /home/ubuntu/Documents/PADM_project/padm-project-2022f/src/../models/ycb/004_sugar_box/textured.obj
Loading /home/ubuntu/Documents/PADM_project/padm-project-2022f/src/../models/ycb/010_potted_meat_can/textured.obj
[+] Generated activity plan:
['drive_to_counter', 'open_drawer', 'pick_up spam', 'place_in_drawer spam', 'close_drawer', 'pick_up sugar', 'place_on_counter sugar']
[*] Thinking...
[+] Path found!
[*] Thinking...
[+] Path found!
[*] Thinking...
[+] Path found!
[*] Thinking...
[+] Path found!
[*] Thinking...
[+] Path found!
[*] Thinking...
[*] Thinking...
[+] Path found!
[*] Thinking...
[+] Path found!
...
```
</details>

**5. You can also mention any challenges you faced and things that you tried irrespective of whether that worked or not.**
- We struggled to implement automated pybullet approaches to grasp generation with our implementation. Ultimately, we chose to design own set grasp poses based on visual consistency with the known poses of each object.
- The pairwise collision checking method we used required updates to the world state, which would be visualized and result in chaotic arm motion while RRT was executing. In order to improve our visualization without worsening RRT performance by preserving world states, we simply pause the visualizer while RRT is executing and resume once a trajectory has been returned. This also improved the execution speed of RRT significantly. 
- We decided to not implement RRT with regards to the motion of the wheeled robot base due to conceptual challenges among other reasons. In order for the robot base motion to be physically consistent, the rotation and driving motions are dependant on one another (the robot must turn in a direction and then drive forwards, rather than translating freely). We were unsure how to best encode this task for our RRT solver while also ensuring that the resulting trajectory produced a compelling visual. Due to it being known that there were no obstacles between the initial and goal states of the robot base, we felt it was acceptable to avoid RRT and instead compute subsequent turning, driving, and turning trajectories to execute. 
- At first, the position of held objects would be modified with motion of the manipulator arm but not the object orientation. We found there were edge case collisions when simplifying the motion in this manner, so the code was adapted to perform more realistic full rigid body transforms of the held object during arm motion. 

## Trajectory Optimization (will be updated by end of 12/14!)

1. Explain the files, key functions, and the solver you used.

We attempted to use the SNOPT solver via pydrake.

2. Explain what optimization problem you are trying to solve and why

The problem we are solving is moving the robot gripper from a start point to goal point. The optimization must account for the time required to move from the start point to the goal point, as well as the arbitrarily-defined step size we used in the initial motion planning. In light of our solution to motion planning, we want to optimize over the joint angles of the robot arm.

3. Formalize the constrained optimization problem using mathematical symbols and relationships 

The optimization problem we are trying to solve is essentially the same one represented here: https://manipulation.csail.mit.edu/trajectories.html#section2


4. Mention any challenges you faced and things that you tried irrespective of whether that worked or not. This will be very important if the problem you defined doesn’t end up working. 

5. GIF/video of the robot executing the plan and embedded in the README

6. Compare the resulting optimized trajectory to the initial sample-based motion plan


## Conclusion

### Reflections

### Contributions

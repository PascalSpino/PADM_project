# PADM_project
Repository for Final Project in Principles of Autonomy and Decision Making

DELIVERABLE 1:

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

We followed the documentation available on Planning.wiki (https://planning.wiki/ref/pddl/problem) to generate our own domain and problem PDDL files. The documentation covered a different problem, but based on how the two files were constructed we were able to adapt to this project. Our approach was to develop the simplest possible set of predicates and actions to achieve planning for this problem, rather than develop predicates and actions for a more general case. 

**3. You can also mention any challenges you faced and things that you tried irrespective of whether they worked or not**

There are several versions of PDDL and the syntax varies slightly between them. We focused primarily on PDDL 2.1 but suspected that we would later have to make significant modifications in order to be parsed successfully. At first, we tried to craft our predicates with objects for the drawer, countertop, gripper, spam can, sugar box, etc… But we instead reduced our set of objects to just the spam can and sugar box, and crafted predicates to describe the drawer state and object locations without additional objects. 

DELIVERABLE 2:

**1. Qualitatively mention your assumptions that you made about the about the environment, state space, and start and goal positions**
- The environment is static except for the movement of the arm itself
- The state space obstacles are the kitchen objects

**2. Explain the files and the motion planners you implemented**
- Still in progress
- Robot is given a goal grasp pose, and performs RRT by sampling poses with bias towards the goal pose. Each generated pose is checked for collision and discarded if so

**3. Explain how you integrated the activity plan with the motion plan**
- The activity planner outputs a file that the motion planner can read from, which each string "command" in the file corresponding to a function that makes the robot perform this action in simulation.

**4. GIF/video of the robot executing the plan**
- Executing part of the plan: https://youtu.be/AJd9sEfJcZI

**5. You can also mention any challenges you faced and things that you tried irrespective of whether that worked or not.**


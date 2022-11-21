# PADM Final Project

Documentation of all products developed for the Final Project of Principles of Autonomy and Decision Making (6.4132/16.412).

## DELIVERABLE 1: 

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

We followed the documentation available on Planning.wiki (https://planning.wiki/ref/pddl/problem) to generate our own domain and problem PDDL files. The documentation covered a different problem, but based on how the two files were constructed we were able to adapt them to this project. Our approach was to develop the simplest possible set of predicates and actions for path planning specific to this problem, rather than develop predicates and actions for a more general case. 

**3. You can also mention any challenges you faced and things that you tried irrespective of whether they worked or not**

There are several versions of PDDL and the syntax varies slightly between them. We focused primarily on PDDL 2.1 but suspected that we would later have to make significant modifications in order for our .pddl files to be parsed successfully. At first, we tried to craft our predicates with objects for the drawer, countertop, gripper, spam can, sugar box, etc. Ultimately, we instead reduced our set of objects to just the spam can and sugar box, and crafted predicates to describe the drawer state and object locations without additional objects. 

## DELIVERABLE 2: 

**1. Qualitatively mention your assumptions that you made about the about the environment, state space, and start and goal positions**
- The environment is static except for the movement of the arm itself
- The state space obstacles are the kitchen objects
- All objects within the environment will always have the same starting position (i.e. the sugar box and spam can will always be found at the same 3-dimensional point)
- We are free to hard code waypoints within the environment for the purpose of charting the robot's motion and the arm/gripper's motion


**2. Explain the files and the motion planners you implemented**
- The `domain.pddl` file contains our final domain specifications. We had four predicates that can be applied to objects, including `on_counter ?x`, `in_drawer ?x`, `on_burner ?x`, and `gripper_holding ?x`. We had two states to represent the drawer (`drawer_open` and `drawer_closed`), two to represent the robot's location relative to the counter (`robot_far_from_counter` and `robot_close_to_counter`, and one to represent the state of the gripper (`gripper_empty`). For operations with these predicates, we defined the following actions: `pick_up`, `place_in_drawer`, `place_on_counter`, `open_drawer`, `close_drawer`, and `drive_to_counter`. By design, only the first three of these actions requre an object to be provided. We chose to omit using the drawer as an object, and thus the open/close drawer actions did not require provision of an object.
- The `problem.pddl` file contains our problem in PDDL format. We defined the problem as having only two objects, the ones we want to move to new locations, `spam` and `sugar`. We then defined the initial state of problem as specified in the project specifications, with the following predicates applied: `on_burner sugar`, `on_counter spam`, `gripper_empty`, `drawer_closed`, and `robot_far_from_counter`. These were sufficient to characterize the initial state. We also defined the goal in problem file, containing the requirements: `on_counter sugar`, `in_drawer spam`, and `drawer_closed`.
- The `planner.py` file contains our activity planner implentation. We used the suggested `pddl-parser` library and a `Breadth First Search` approach. As per expectations, we made our own BFS implentation using Node objects to represent each state. Each Node object contains a instance variable that indicate's its parent (or None for the starting state). After a state containing the goal requirements is found, we generate the path by iterating backwards through the Node parents to determine the order taken from the start state to the goal.
- Motion planner still in progress
- Robot is given a goal grasp pose, and performs RRT by sampling poses with bias towards the goal pose. Each generated pose is checked for collision and discarded if so

**3. Explain how you integrated the activity plan with the motion plan**
- ~~The activity planner outputs a file that the motion planner can read from, which each string "command" in the file corresponding to a function that makes the robot perform this action in simulation.~~
- We added an instance variable to the Planner class in `planner.py` so that the plan could be directly accessible without the need to read from a file. With this approach we chose to represent the plan as an ordered list where each element is a string containing the next step in the plan. After the `solve()` method is run, the instance variable `plan` of a `Planner` object will contain the BFS-generated plan: `['drive_to_counter ', 'pick_up sugar', 'place_on_counter sugar', 'open_drawer ', 'pick_up spam', 'place_in_drawer spam', 'close_drawer ']`

**4. GIF/video of the robot executing the plan**
- Executing part of the plan: https://youtu.be/AJd9sEfJcZI

**5. You can also mention any challenges you faced and things that you tried irrespective of whether that worked or not.**
- For the activity planning portion, we initially attempted to minimize the number of predicates and actions. We figured keeping things simple was the best approach. This led us to use negations of predicates heavily in both our `domain.pddl` and `problem.pddl` files. This would have worked and been fine theoretically, but it sometimes made it more diffucult to debug the logic before we got our activity planner. Ultimately we made a handful of changes, including only having positive initial predicates, only having positive goals, and including a couple pairs of predicates that that represented opposed states for specific items (i.e. `drawer_open` and `drawer_closed`, as well as `robot_close_to_counter` and `robot_far_from_counter`). Using a couple paired predicates made our `.pddl` files slightly less consise, but a bit easier to debug.

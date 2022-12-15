#!/usr/bin/env python

from PDDL import PDDL_Parser
import sys

class Node:
    '''Class to represent node objects'''

    def __init__(self, state, action=None, parent=None):
        self.state = state
        self.action = action
        self.parent = parent

    def get_state(self):
        ''' Returns node's state.'''
        return self.state

    def get_action(self):
        '''Returns action applied to node's state.'''
        return self.action

    def get_parent(self):
        '''Returns node's parent node.'''
        return self.parent


class Planner:
    '''Finds path to a goal state based on a provided domain and problem.'''

    def __init__(self, domain, problem):
        self.domain = domain
        self.problem = problem
        self.plan = None
    
    def solve(self):
        '''Finds path from init to goal if one exists.'''
        
        # Instantiate PDDL_Parser
        parser = PDDL_Parser()

        # Parse the domain and problem files
        parser.parse_domain(self.domain)
        parser.parse_problem(self.problem)

        # Extract initial state and goals information from parser
        state = parser.state
        goals = parser.positive_goals

        # Check if initial state meets goal criteria
        if goals.issubset(state):
            print("[+] Initial state satisfies all goals!")
            return [state]
        
        # Action space extraction is performed by groundify method from pddl_parser's action.py
        # so that action_space includes all action/object combinations 
        action_space = []
        for action in parser.actions:
            # Add actions paired with items they can modify
            for item in action.groundify(parser.objects, parser.types):
                action_space.append(item)
        

        ##########################################
        ### Conduct Breadth First Search (BFS) ###
        ##########################################
        
        # Instantiate visited list and queue
        visited = set([Node(state)])
        queue = [Node(state)]
        
        # Instantiate counter to record number of states explored
        count = 0

        # Main loop
        while queue:
            # Pop first element off of queue
            node = queue.pop(0)
            # Increment counter
            count += 1
            # Iterate over action_space, exploring paths until reaching a state that satisfies all goals if one exists
            for action in action_space:
                # Check if taking action yields a new state
                if action.positive_preconditions.issubset(node.state) and action.negative_preconditions.isdisjoint(node.state):
                    # Carve out new_state based on add effects and delete effects
                    new_state = node.state.difference(action.del_effects)
                    new_state = new_state.union(action.add_effects)
                    # Build new_node from new_state, recording action taken and parent node
                    new_node = Node(new_state, action, node)
                    # Check whether new_node has not been visited, if not, visit, then add to visited list and queue
                    if new_node not in visited:
                        # Check if new_code contains goal criteria
                        if goals.issubset(new_node.state):
                            print(f"[+] Found plan after {count} node expansions.")
                            # Extract and return plan
                            plan = []
                            while new_node.parent != None:
                                plan.insert(0, new_node.action)
                                new_node = new_node.parent
                            # Assign plan to plan instance variable 
                            self.plan = [(action.name + " " + " ".join(action.parameters)).strip() for action in plan] 
                            return plan
                        # Add new nodes to visited list and queue
                        visited.add(new_node)
                        queue.append(new_node)
        
        # Return None if no path was found
        print("[-] No path found.")

        return None

    
if __name__ == "__main__":
    # Get domain and problem filenames from command line
    domain = sys.argv[1]
    problem = sys.argv[2]

    # Instantiate planner
    planner = Planner(domain, problem)
    
    # Solve plan
    plan = planner.solve()

    # Format actions in plan as list
    plan_actions = [action.name + " " + " ".join(action.parameters) for action in plan]
    
    # Print action to console
    print("[+] Action sequence to complete both tasks:\n")
    for i in range(len(plan_actions)):
        print(f"Step {i+1}:", plan_actions[i])

    print("\n[+] As a list:")
    print(planner.plan)
    

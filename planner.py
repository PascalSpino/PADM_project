# This code is a rough sketch of fast forward heuristic planning
# It is not tested and serves more as a sketch of an approach than
# anything else

from .PDDL import PDDL_Parser

# Rough sketch of fast forward heuristic planning
class Planner:
    def solve(self, domain, problem):
        # Instantiate arser
        parser = PDDL_Parser()
        parser.parse_domain(domain)
        parser.parse_problem(problem)
        
        # Parse data
        start = parser.init
        goal = parser.goal

        # Apply fast forward heuristic to generate plan
        visited = set([init])
        queue = [init]
        while queue:
            state = queue.pop(0)
            for action in actions:
                new_states = action.expand # state expansion should yield states reachable from current state
                for new_state in new_states:
                  visited.add(new_state)
       
        path = 'TODO: extract path from init to goal'
        
        return path

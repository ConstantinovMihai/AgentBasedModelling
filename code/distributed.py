"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from aircraft import AircraftDistributed
from cbs import detect_collision, detect_collisions

class DistributedPlanningSolverIndividual(object):
    """A distributed planner where agents do not communicate with each other"""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.CPU_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristics = [] 
        self.radar_radius = 5
        # the locations of all agents
        self.locations = starts

        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

        self.time = 0 # this is going to incrementaly increase and decisions are going to be made at each timestep
    
    def goalsReached(self):
        """ Checks whether the agents reached their goals
            Returns a lists of bools 
        """
        reached = []
        for idx, ag in enumerate(self.locations):
            reached.append(ag == self.goals[idx])

        return reached


    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        # stores all the paths
        
        # in the prioritized planning, the entire path was compute at once
        # now, for each time step we should 1. update the constraint list for the next timestep; use the radar
        # check if new constraints were added and then find the next location using A*
        
        # this stores the paths for each agent (this time we'll append the results with one location at a time)
        result = []
        
        # stopping criterion for the simulation
        # iterates over time
        # iterates among all the agents 
        # setting up the bubble constraints
        
   

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, [])
            if path is None:
                raise BaseException('No solutions')
            result.append(path)


        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))

        return result, self.CPU_time


class DistributedPlanningSolver(object):
    """A distributed planner"""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.CPU_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristics = []
        # T.B.D.
        
    def find_solution(self):
        """
        Finds paths for all agents from start to goal locations. 
        
        Returns:
            result (list): with a path [(s,t), .....] for each agent.
            self.CPU_Time (float) : how much time the solver needs
        """
        # Initialize constants       
        start_time = timer.time()
        result = []
        self.CPU_time = timer.time() - start_time
        
        
        # Create agent objects with AircraftDistributed class
        for i in range(self.num_of_agents):
            newAgent = AircraftDistributed(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i)
        
        
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)
        
        return result , self.CPU_time # Hint: this should be the final result of the distributed planning (visualization is done after planning)
"""
This file contains the implemention of distributed planning WITHOUT COORDINATION.
"""

import imp
import re
import numpy as np
import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from aircraft import AircraftDistributed
from cbs import detect_collision, detect_collisions
from distributed_class import DistributedPlanning


class DistributedPlanningSolverIndividual(DistributedPlanning):
    """A distributed planner where agents do not communicate with each other"""
    
    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        # stores all the paths
        
        # in the prioritized planning, the entire path was computed at once
        # now, for each time step we should 1. update the constraint list for the next timestep; use the radar
        # check if new constraints were added and then find the next location using A*
        
        agents = []

        # Create agent objects with AircraftDistributed class
        for i in range(self.num_of_agents):
            newAgent = AircraftDistributed(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i)
            
            agents.append(newAgent)

        # this stores the paths for each agent (this time we'll append the results with one location at a time)
        result = []

        # start location of agents need to be added to paths
        for agent in agents:
            agent.path.append(agent.start)

        # simulate until all the agents reached their goals
        while not all(self.goalsReached(agents)):
           
            # iterate for each agent
            # create constraints which will be used to run planning for each agent
            for agent in agents:
                
                # stores the locations of nearby agents
                prox_loc = self.radarScanner(agent, agents)
                
                # generates constraints using the prox_loc and the bubble method
                agent.addBubbleConstraints(self.time, prox_loc)
                
            # run planning for each agent
            for agent in agents:                
                path = a_star(agent.my_map, agent.location, agent.goal, agent.heuristics, agent.id, agent.constraints, self.time, True)
                
                # if there is no path, agent waits at current location
                if path is None:                    
                    agent.path.append(agent.location)
                # the next location of the agent is stored as the second element in the path planned by A* (element zeroth is the current location)
                elif (len(path) > 1):
                    agent.location = path[1]
                    agent.path.append(path[1])
                else: # the agent reached its goal
                    agent.location = path[0]
                    agent.path.append(path[0])
                
                
            # increment time
            self.time += 1

        # when everything is done, store the final paths in the results 
        for agent in agents:
            result.append(agent.path)

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))

        return result, self.CPU_time

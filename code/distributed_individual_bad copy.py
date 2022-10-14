"""
This file contains the implemention of distributed planning WITHOUT COORDINATION.
"""

import numpy as np
import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, is_constrained
from aircraft import AircraftDistributed
from cbs import detect_collision, detect_collisions
from distributed_class import DistributedPlanning
from single_agent_planner import build_constraint_table, is_constrained


class DistributedPlanningSolverIndividual(DistributedPlanning):
    """A distributed planner where agents do not communicate with each other"""
    
    def constraintsInPath(self, path, agent):
        """ check whether there exists constraints in the path of an agent

        Args:
            path (list): the planned path for the agent
            agent (AircraftDistributed): the agent object 
        """
        # build constraint table for this agent
        indexed_constraint_table = build_constraint_table(agent.constraints, agent.id)
        for idx, loc in enumerate(path):
            if idx < len(path) - 1:
                if is_constrained(loc, path[idx + 1], self.time + idx, indexed_constraint_table):
                    return True

        # no constraints in the way 
        return False


    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        # stores all the paths
        
        # in the prioritized planning, the entire path was computed at once
        # now, for each time step we should 1. update the constraint list for the next timestep; use the radar
        # check if new constraints were added and then find the next location using A*
        
        agents = []
        
        # the paths of the agents are stored here
        # if, when computing the paths, no constraints is in the path
        # there is no point in computing A* again
        # initiliase the paths array
        paths = []

        # Create agent objects with AircraftDistributed class
        for i in range(self.num_of_agents):
            newAgent = AircraftDistributed(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i)
            agents.append(newAgent)
            paths.append([])

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
            for idx, agent in enumerate(agents):  
                              
                # run A* to check for the best path (at this iteration)
                # NOTE: run A* only if there are constraints on the preexisting path
                # or there is not path yet
                # if not paths[idx]:
                #     paths[idx] = a_star(agent.my_map, agent.location, agent.goal, agent.heuristics, agent.id, agent.constraints, self.time, True)
                #     print("WTF")
                # print(f"paths are {paths}")    
                # if self.constraintsInPath(paths[idx], agent):
                #     print("it is hit in here")
                paths[idx] = a_star(agent.my_map, agent.location, agent.goal, agent.heuristics, agent.id, agent.constraints, self.time, True)
                    
                    # update the path for the agent  

                # if there is no path, agent waits at current location
                if paths[idx] is None:                    
                    agent.path.append(agent.location)

                # the next location of the agent is stored as the second element in the path planned by A* (element zeroth is the current location)
                elif (len(paths[idx]) > 1):
                    agent.location = paths[idx][1]
                    agent.path.append(paths[idx][1])

                else: # the agent reached its goal
                    agent.location = paths[idx][0]
                    agent.path.append(paths[idx][0])
                
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

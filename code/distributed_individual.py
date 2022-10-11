"""
This file contains the implemention of distributed planning WITHOUT COORDINATION.
"""

import re
import numpy as np
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

    def goalsReached(self, agents):
        """ Checks if all agents reached their goals

        Args:
            agents (list): list with the agent objects
        Returns a list with bool values (did the agent reached its goal?)
        """

        reached_goal = []
        for agent in agents:
            reached_goal.append(agent.location == agent.goal)

        return reached_goal


    def radarScanner(self, start_agent, agents, radar_radius = 5):
        """ Starting from the start agent, it scans the map and finds all agents (and their locations) within a fixed radius distance

        Args:
            start_agent (AircraftDistributed): scanning agent
            agents (list of AircraftDistributed): list of agents  
            radar_radius (int, optional): The radius at which the scan is performed. Defaults to 5.
        """
        
        def distanceAgents(loc1, loc2):
            """ Helper function which computes the (euclidian) distance between two agents

            Args:
                loc1 (tuple): location of agent 1
                loc2 (tuple): location of agen2
            Returns a float, containing the (euclidian) distance between the (two) agents
            """

            return np.sqrt((loc1[0] - loc2[0]) * (loc1[0] - loc2[0]) + (loc1[1] - loc2[1]) * (loc1[1] - loc2[1]))


        start_loc = start_agent.location
        prox_loc = []
        # creates a list with all other agents' locations
        for agent in agents:
            if agent.id != start_agent.id:
                # compute the distance between the starting agent and all the others, and store the locations of the ones
                #  who are in the proximity of the starting agent (distance < radius)
                if distanceAgents(start_agent.location, agent.location) < radar_radius:
                    prox_loc.append(agent.location)

        return prox_loc




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

        # simulate until all the agents reached their goals
        while not all(self.goalsReached(agents)):
            # iterate for each agent
            for agent in agents:
                # stores the locations of nearby agents
                prox_loc = self.radarScanner(agent, agents)
                # generates constraints using the prox_loc and the bubble method
                agent.addBubbleConstraints(self.time, prox_loc)
                path = a_star(agent.my_map, agent.location, agent.goal, agent.heuristics, agent, agent.constraints)
                # if there is no path it means there are no solutions
                if path is None:
                    raise BaseException('No solutions')
                # the next location of the agent is stored as the second element in the path planned by A* (element zeroth is the current location)
                agent.location = path[1]
                agent.path.append(path[1])
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

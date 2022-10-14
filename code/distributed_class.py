"""
This file contains the implemention of distributed planning WITHOUT COORDINATION.
"""

import re
import numpy as np
import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from aircraft import AircraftDistributed
from cbs import detect_collision, detect_collisions
from single_agent_planner import is_constrained, build_constraint_table

class DistributedPlanning(object):
    """ parent class for the planners
    Args:
        object (_type_): _description_
    """

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
    
    def constraintsInPath(self, path, agent):
        """ check whether there are constraints in the path of an agent
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


    def radarScanner(self, start_agent, agents, radar_radius = 4):
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
                # along with location, radar returns whether agent has reached its goal or not
                if distanceAgents(start_agent.location, agent.location) < radar_radius and agent.location == agent.goal:
                    prox_loc.append({'location':agent.location,'planned_path':agent.planned_path,'reached_goal':True})
                elif distanceAgents(start_agent.location, agent.location) < radar_radius:
                    prox_loc.append({'location':agent.location,'planned_path':agent.planned_path,'reached_goal':False})

        return prox_loc

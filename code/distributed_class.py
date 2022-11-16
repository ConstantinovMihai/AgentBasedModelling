"""
This file contains the implemention of distributed planning WITHOUT COORDINATION.
"""

import numpy as np
import time as timer
from single_agent_planner import computeHeuristics, a_star, getSumOfCost
from aircraft import AircraftDistributed
from cbs import detectCollision, detectCollisions
from single_agent_planner import isConstrained, buildConstraintTable

class DistributedPlanning(object):
    """ parent class for the planners_
    """

    def __init__(self, my_map, starts, goals, heuristics):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        heuristics - [h1, h2, h3, h4] list of heuristisc
        """
        self.CPU_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristics = []
        self.current_heuristics = []
        
        # the locations of all agents
        self.locations = starts

        # heuristics 
        
        # the factor by which the wait time increases the heuristic value for the cell the agent is waiting on
        self.wait_time_factor = heuristics[0]
        # the factor by which the heuristic value of a cell with a hard constraint on it increases
        # this applies to cells where agents have reached their goal
        self.hard_heur_factor = heuristics[1]
        # the factor by which the heuristic value of a cell with a hard constraint on it increases
        self.soft_heur_factor = heuristics[2]
        # the number of time steps of planned path the agents broadcast to each other
        self.plan_broadcast = heuristics[3]
        # the distance an agent can see
        self.radar_radius = heuristics[4]
        for goal in self.goals:
            self.heuristics.append(computeHeuristics(my_map, goal))

        self.time = 0 # this is going to incrementaly increase and decisions are going to be made at each timestep

    
    def goalsReached(self, agents):
        """ Checks if all agents reached their goals

        Args:
            agents (list): list with the agent objects
        Returns a list with bool values (did the agent reach its goal?)
        """

        reached_goal = []
        for agent in agents:
            reached_goal.append(agent.location == agent.goal)

        return reached_goal


    def radarScanner(self, start_agent, agents):
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

        prox_loc = []
        # creates a list with all other agents' locations
        for agent in agents:
            if agent.id != start_agent.id:
                # compute the distance between the starting agent and all the others, and store the locations of the ones
                #  who are in the proximity of the starting agent (distance < radius)
                # along with location, radar returns whether agent has reached its goal or not
                # also whether the neighbour agent is blocked from its final goal
                # also stores the neighbour id
                # and the distance the opponent has to its goal
                if distanceAgents(start_agent.location, agent.location) < self.radar_radius and agent.location == agent.goal:
                    prox_loc.append({'location':agent.location,'planned_path':agent.planned_path,'reached_goal':True, 'blocked':agent.blockage, 'opponent_id': agent.id, 'opponent_dist_to_goal': agent.heuristics[agent.location]})
                elif distanceAgents(start_agent.location, agent.location) < self.radar_radius:
                    prox_loc.append({'location':agent.location,'planned_path':agent.planned_path,'reached_goal':False, 'blocked':agent.blockage, 'opponent_id' : agent.id, 'opponent_dist_to_goal': agent.heuristics[agent.location]})

        return prox_loc

    def initialiseAgents(self):
        """ Create agent objects with AircraftDistributed class
        """
        agents = []

        for i in range(self.num_of_agents):
            newAgent = AircraftDistributed(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i)
            agents.append(newAgent)
        
        # start location of agents need to be added to paths
        for agent in agents:
            agent.path.append(agent.start)

        return agents 


    def waitingTime(self, agent):
        """ Computes how many timesteps an agent has been waiting
        Args: agent (AircraftDistributed) : the agent object
        Returns the total time spent waiting (int)
        """
        wait_time = 0
        while True:
            # if an agent starts at its goal and just stays there
            if wait_time + 1 >= len(agent.path):
                break
            # if an agent's location at a time step is different than the location at the prev timestep, break
            if agent.path[-wait_time - 1] != agent.path[-wait_time - 2]:
                break
            
            wait_time += 1

        agent.waiting = wait_time


        return wait_time

        
    def printCollisions(self, paths):
        """ Prints all the detected collisions in a list of paths

        Args:
            paths (list): list with the paths of agents
            map (str) : the name of the map tested
        """

        collisions = detectCollisions(paths)
        
        if collisions:
            print(f"Collisions detected in a map defined by: my_map {self.my_map}\n starts {self.starts}\n and goals {self.goals}")
            print(collisions)
            raise Exception('Collision')


    def printResult(self, result):
        """ Prints the total cost of the simulation, as well as info about the CPU time

        Args:
            results (int): total cost of the simulation
        """

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(getSumOfCost(result)))

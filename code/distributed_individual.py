"""
This file contains the implemention of distributed planning WITHOUT COORDINATION.
"""

import imp
import re
import numpy as np
import time as timer
from single_agent_planner import computeHeuristics, a_star, getSumOfCost
from aircraft import AircraftDistributed
from cbs import detectCollision, detectCollisions
from distributed_class import DistributedPlanning


class DistributedPlanningSolverIndividual(DistributedPlanning):
    """A distributed planner where agents do not communicate with each other"""
    

    def modifyHeuristics(self, agent):
        """ Modifies the heuristic values of cells where a constraint is imposed for a certain agent
        """
        # TODO: check if clearing or recomputing the heuristics affects the behaviours of the model
        #agent.heuristics = computeHeuristics(agent.my_map, agent.goal)
        for constraint in agent.constraints:
            if constraint['loc'][0] in agent.heuristics:
                if constraint['hard']:
                    #TODO: this cell will be occupied forever as the agent reached its goal
                    agent.heuristics[constraint['loc'][0]] += 0
                else:
                    # TODO: tune the parameters (the cell might become free in the future) 
                    agent.heuristics[constraint['loc'][0]] += 0


    def appendPlannedPath(self, agent, path):
        """_summary_

        Args:
            agent (AircraftDistributed): the agent whose path is going to be changed
            path (list) : the temporary planned path as computed by A*
        """
         # if there is no path, agent waits at current location
        if path is None:                    
            agent.planned_path.append(agent.location)
        # the next n locations of the agent as planned by A*
        elif (len(path) > 1):            
            agent.planned_path = path[1:3]
        else: # the agent reached its goal
            agent.planned_path = path


   

    def getNextLocations(self, agents):
        """ Gets the list of all the agents' next intended location
 
        Args:
            agents (list of AircraftDistributed): the list of the agents objects
        """
        next_locations = []
        for agent in agents:
            next_locations.append([agent.planned_path[0]])
        
        return next_locations


    def collisionHandling(self, agents):
        """ Identifies agent who plan to occupy the same cells in the next time step

        Args:
            agents (list of AircraftDistributed): the list of the agents objects
        """
        next_locations = self.getNextLocations(agents)
        collisions = detectCollisions(next_locations)
        
       
        for agent in agents:
                collided = False
                # for each agent check if any of the collisions belong to it 
                for collision in collisions:                
                    if agent.id == collision['a1']:
                        # if the agent has a collision the first agent waits a timestep
                        # TODO: decide over a tiebreaker
                        agent.path.append(agent.location)
                        collided = True
                        break
                # if there are no collisions, append the intended location to the path
                # and update the current location of the agent
                if not collided:
                    agent.path.append(agent.planned_path[0])
                    agent.location = agent.planned_path[0]


    def appendFinalPaths(self, agents, result):
        """ when everything is done, store the final paths in the results and trim the paths

        Args:
            agents (list of AircraftDistributed): the list of the agents objects
            result (list) : the list of final paths
        """
        
        # when everything is done, store the final paths in the results and remove locations of agent waiting at goal from path to not influence the total cost
        for agent in agents:          
            trim_length = 0
            found = False
            while found == False:
               
                if trim_length +1 == len(agent.path):
                    found = True
                    continue
                if agent.path[-trim_length-1] != agent.path[-trim_length-2]:
                    found = True
                else:
                    trim_length += 1
            path2 = agent.path[:len(agent.path)-trim_length].copy()

            result.append(path2)


    def findSolution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        # in the prioritized planning, the entire path was computed at once
        # now, for each time step we should 1. update the constraint list for the next timestep; use the radar
        # check if new constraints were added and then find the next location using A*
        agents = self.initialiseAgents()
        # this stores the paths for each agent (this time we'll append the results with one location at a time)
        result = []

        # simulate until all the agents reached their goals
        while not all(self.goalsReached(agents)):
    
            # create constraints which will be used to run planning for each agent
            for agent in agents:
                # stores the locations of nearby agents
                prox_loc = self.radarScanner(agent, agents)
                # generates constraints using the prox_loc and the bubble method
                agent.addConstraints(self.time, prox_loc)
                
            # run planning for each agent
            for idx, agent in enumerate(agents):
                agent.planned_path = []
                wait_time = self.waitingTime(agent)
                
                # h_value of wait location is increased by time spent waiting 
                agent.heuristics[agent.path[-1]] += (wait_time)
                self.modifyHeuristics(agent)

                # update the planned path
                path = a_star(agent.my_map, agent.location, agent.goal, agent.heuristics, agent.id, agent.constraints, self.time, True)
                self.appendPlannedPath(agent, path)

            # handle the possible collision situations       
            self.collisionHandling(agents)

            # increment time
            self.time += 1

        self.appendFinalPaths(agents, result)

        self.CPU_time = timer.time() - start_time
        self.printResult(result)

        return result, self.CPU_time
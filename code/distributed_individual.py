"""
This file contains the implemention of distributed planning WITHOUT COORDINATION.
"""

import random
from turtle import st
import numpy as np
import time as timer
from single_agent_planner import a_star
from aircraft import AircraftDistributed
from cbs import detectCollisions
from distributed_class import DistributedPlanning

class DistributedPlanningSolverIndividual(DistributedPlanning):
    """A distributed planner where agents do not communicate with each other"""
    
    def manuallySetEnvironment(self, my_map, starts, goals):
        """ Manually set the map, starts and goals, useful for debugging purposes

        Args:
            my_map (string): the map set as a list of lists of Bools
            starts (list): the list of start locations
            goals (list): the list of goal locations
        """
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        

    def modifyHeuristics(self, agent, hard_heur_factor, soft_heur_factor):
        """ Modifies the heuristic values of cells where a constraint is imposed for a certain agent
        """
        # TODO: check if clearing or recomputing the heuristics affects the behaviours of the model
        #agent.heuristics = computeHeuristics(agent.my_map, agent.goal)
        agent.current_heuristics = agent.heuristics.copy()
        for constraint in agent.constraints:
            if constraint['loc'][0] in agent.heuristics:
                if constraint['hard']:
                    #TODO: this cell will be occupied forever as the agent reached its goal
                    agent.current_heuristics[constraint['loc'][0]] = hard_heur_factor * agent.heuristics[constraint['loc'][0]]
                else:
                    # TODO: tune the parameters (the cell might become free in the future) 
                    agent.current_heuristics[constraint['loc'][0]] = soft_heur_factor * agent.heuristics[constraint['loc'][0]]


    def appendPlannedPath(self, agent, path,plan_broadcast):
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
            agent.planned_path = path[1:plan_broadcast]
        else: # the agent reached its goal
            agent.planned_path = path


   
    def getNextLocations(self, agents : list):
        """ Gets the list of all the agents' next intended location
 
        Args:
            agents (list of AircraftDistributed): the list of the agents objects
        """
        next_locations = []
        for agent in agents:
            next_locations.append([agent.planned_path[0]])
        
        return next_locations


    def collisionHandling(self, agents : AircraftDistributed):
        """ Identifies agent who plan to occupy the same cells in the next time step

        Args:
            agents (list of AircraftDistributed): the list of the agents objects
        """

        def assignPriority(collision : dict, waiting_times : list):
            """ Helper function which decides which agent in a conflict has priority, based on the duration spent waiting by the agents
            
            Args:
                collision (dicts): dictionary containing the indeces of the agents, the location and the timestep of the collision
                waiting_times (list): list with the time spent waiting by the agents
            Returns the index of the agent who was assigned priority
            """
            # give the priority to the agent who waited the most so far
            # this is the tie-breaker for collisions
            priority = collision['a1']

            #print(waiting_times[collision['a1']])
            #print(waiting_times[collision['a2']])

            if (waiting_times[collision['a1'] > waiting_times[collision['a2']]]):
                priority = collision['a2']
            
            # TODO: see if we can make this work better than the previous
            # if (waiting_times[collision['a1'] == waiting_times[collision['a2']]]):
            #     priority = random.choice([collision['a1'], collision['a2']])
            
            return priority


        next_locations = self.getNextLocations(agents)
        collisions = detectCollisions(next_locations)
        
        waiting_times = []
        for agent in agents:
            waiting_times.append(agent.waiting)
       
        for agent in agents:
           
            collided = False
            # for each agent check if any of the collisions belong to it 
            for collision in collisions:       
                
                # give the priority to the agent who waited the most so far
                # this is the tie-breaker for collisions
                priority = assignPriority(collision, waiting_times)

                if agent.id == priority:
                    # if the agent has a collision the non-priority agent waits a timestep
                    agent.path.append(agent.location)
                    collided = True
                    break

            # if there are no collisions, append the intended location to the path
            # and update the current location of the agent
            if not collided:
                agent.path.append(agent.planned_path[0])
                agent.location = agent.planned_path[0]


    def appendFinalPaths(self, agents, result):
        """ When everything is done, store the final paths in the results and trim the paths

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
        while not all(self.goalsReached(agents)) and self.time<500:
            
            if self.time == 499:
                print(f"time limit hit in a map defined by: my_map {self.my_map}\n starts {self.starts}\n and goals {self.goals}")

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
                agent.heuristics[agent.path[-1]] += (self.wait_time_factor * wait_time)
                self.modifyHeuristics(agent, self.hard_heur_factor, self.soft_heur_factor)

                # update the planned path
                path = a_star(agent.my_map, agent.location, agent.goal, agent.current_heuristics, agent.id, agent.constraints, self.time, True)
                
                self.appendPlannedPath(agent, path, self.plan_broadcast)
<<<<<<< HEAD
                #print(self.time)
                #print(agent.id)
                #print(path)
=======

                #print('agent',agent.id)
                #print(path)
                
>>>>>>> dce9638b1e8ecd005b5ec9122a7b29db5f119011

            # handle the possible collision situations       
            self.collisionHandling(agents)

            # increment time
            self.time += 1

        self.appendFinalPaths(agents, result)

        self.CPU_time = timer.time() - start_time
        # check whether there are collisions
        
        self.printCollisions(result)    
        self.printResult(result)

        return result, self.CPU_time
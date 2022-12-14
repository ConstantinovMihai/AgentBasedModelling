"""
This file contains the implemention of distributed planning WITHOUT COORDINATION.
"""

import math
import random
import numpy as np
import time as timer
from single_agent_planner import a_star, computeHeuristics
from aircraft import AircraftDistributed
from cbs import detectCollisions
from distributed_class import DistributedPlanning
import copy

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


    def appendPlannedPath(self, agent, path,plan_broadcast):
        """update the planned path of the agent

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


    def blockHandling(self, agents : AircraftDistributed):
        """ Identifies agents who are blocking other agents and makes them move

        Args:
            agents (list of AircraftDistributed): the list of the agents objects
        """

        # gather the intended next locations of each agent
        next_locations = self.getNextLocations(agents)
        # detect collisions in the intended next locations
        collisions = detectCollisions(next_locations)

        # create list which keeps track of agents which are blocked
        blocked_agents = []
        for agent in agents:
            blocked_agents.append(agent.blockage)
       
        for agent in agents:        
            # for each agent check if any of the collisions belong to it
            for collision in collisions:
                # if an agent who reached their goal blocks the path of another agent, make it move out of the way
                # a check is performed to verify that another agent is indeed blocked
                if agent.goal == agent.location and (True in blocked_agents):
                    # if this collision belongs to the blocking agent
                    if agent.id == collision['a1'] or agent.id == collision['a2']:
                        #constrain the agent goal for the next time step to force it to move away
                        agent.constraints.append({'agent': agent.id,'loc': [agent.location],'timestep': self.time + 1, 'hard':False})
                        for goal in self.goals:
                            
                            agent.constraints.append({'agent': agent.id,'loc': [goal],'timestep': self.time + 1, 'hard':False})
                        # recalculate path
                        path = a_star(agent.my_map, agent.location, agent.goal, agent.current_heuristics, agent.id, agent.constraints, self.time, True)               
                        # update the agents intended path
                        self.appendPlannedPath(agent, path, self.plan_broadcast)

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
            agent_forced_to_move = collision['a1']

            if (waiting_times[collision['a1'] > waiting_times[collision['a2']]]):
                agent_forced_to_move = collision['a2']      
            
            return agent_forced_to_move

        #gather intended next locations and detect collisions
        next_locations = self.getNextLocations(agents)
        collisions = detectCollisions(next_locations)

        #create list of waiting times
        waiting_times = []
        for agent in agents:
            waiting_times.append(agent.waiting)
        
        counter = 0
        while len(collisions)>0 and counter<25:
            # while there are collisions detected and the counter has not been exhausted
            # in case counter is exhausted, agents likely want to exchange positions and thus making one of the two agents wait, will never resolve the conflict.
            # in this case both agents must be forced to wait
            for agent in agents:                
                for collision in collisions:
                    # give the priority to the agent who waited the most so far
                    agent_forced_to_move = assignPriority(collision, waiting_times)
            
                    if agent.id == agent_forced_to_move:
                        # if the agent has a collision the non-priority agent waits a timestep
                        agent.planned_path[0]=(agent.location)
                        # in case one agent has two collisions associated with it, the loop needs to be exited in order not to make the agent wait twice in one timestep
                        break
                    if counter == 24:
                        for agent in agents:
                            # if the counter has been exhausted and the collision belongs to the agent, force agent to wait
                            # this is done for both agents
                            if agent.id == collision['a1'] or agent.id == collision['a2']:
                                agent.planned_path[0]=(agent.location)                     

            # perform same collision check and increase counter
            next_locations = self.getNextLocations(agents)
            collisions = detectCollisions(next_locations)
            counter+=1

        # once collision detection loop has been exited, update locations of all agents
        for agent in agents:
            agent.path.append(agent.planned_path[0])
            agent.location = agent.planned_path[0]


    def appendFinalPaths(self, agents, result):
        """ When everything is done, store the final paths in the results and trim the paths

        Args:
            agents (list of AircraftDistributed): the list of the agents objects
            result (list) : the list of final paths
        """
        
        # when everything is done, store the final paths in the results and 
        # remove locations of agent waiting at goal from path to not influence the total cost
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
        
    def findBlockages(self, agents):
        """ Find if any agents are blocked from reaching their paths by agents who have already reached their goal
        Args:
            agents (list of AircraftDistributed): the list of the agents objects
        """
        #creates a copy of the default map
        temp_map = copy.deepcopy(self.my_map)                

        for agent in agents:
            # for each agent, if they have reached their goal, that map location is blocked as if it was a wall
            if agent.location == agent.goal:
                temp_map[agent.location[0]][agent.location[1]] = True
        
        # the counter is used to avoid a weird edge case where two agents are blocked, which produces collisions
        counter = 0
        for agent in agents:
            agent.blockage = False
            if agent.location != agent.goal:
                # for each agent if they havent reached their goal, the heuristics is calculated
                # in the case where their current location is not in the list of locations which have a heuristic value, this means there is no path
                # then the agents blockage status is set to true
                heuristics = computeHeuristics(temp_map, agent.goal)
                if agent.location not in heuristics and counter == 0:
                    agent.blockage = True
                    counter +=1
                    # print(agent.id,"Blockage") 

    def adjustHeuristics(self, agent, prox_loc):
        """ 
        Modifies the heuristic values of cells where a constraint is imposed for a certain agent
        """
        
        # copy the agents default heuristics to impose penalties for this specific timestep only
        agent.current_heuristics = copy.deepcopy(agent.heuristics)
        for neighbour in prox_loc:
            # if the neighbour has priority
            # priority is based on agent's id unless the neighbour reached its goal
            if neighbour['reached_goal']:                              
                for i, planned_loc in enumerate(neighbour['planned_path']):
                    agent.current_heuristics[planned_loc] = self.hard_heur_factor * agent.heuristics[planned_loc]
        
            elif neighbour['opponent_id'] < agent.id and not neighbour['reached_goal']:
                # impose a penalty on the heuristic value of the cells the neighbour plans to inhibit in the next timesteps                     
                for i, planned_loc in enumerate(neighbour['planned_path']):    
                    agent.current_heuristics[planned_loc] = self.soft_heur_factor * agent.heuristics[planned_loc]

    def findSolution(self, time_limit = 60):
    
        """ Finds paths for all agents from their start locations to their goal locations."""
      
        start_time = timer.time()
        agents = self.initialiseAgents()
        # this stores the paths for each agent. This list is filled once a final solution is found
        result = []
        self.CPU_time = timer.time() - start_time 
        # simulate until all the agents reached their goals. A time limit is also imposed in case the algorithm cannot find a solution
        while not all(self.goalsReached(agents)) and (self.time<100 and self.CPU_time <time_limit): 
            self.CPU_time = timer.time() - start_time 
            
            # Find if any agents are blocked from reaching their goal by other agents who have already reached their goal
            self.findBlockages(agents)
            
            # create constraints which will be used to run planning for each agent
            for agent in agents:
                #the amount of time an agent has spent waiting at a location is calculated
                wait_time = self.waitingTime(agent)
                if wait_time > 2:
                    agent.heuristics[agent.path[-1]] += wait_time *agent.heuristics[agent.path[-1]] 
                # fnds and stores the locations of nearby agents                
                prox_loc = self.radarScanner(agent, agents)
                # generates constraints using the prox_loc
                agent.addConstraints(self.time, prox_loc)
                # adjust penalties to cells which are inhibited by a neigbour agent by increasing the heuristic value of these cells
                # the cells agents intend to inhibit in the future are also penalized based on which agent has priority
                self.adjustHeuristics(agent, prox_loc)

                
            # run planning for each agent
            for agent in agents:
                agent.planned_path = []

                # update the planned path                 
                path = a_star(agent.my_map, agent.location, agent.goal, agent.current_heuristics, agent.id, agent.constraints, self.time, True)
                # the planned path is stored                
                self.appendPlannedPath(agent, path, self.plan_broadcast)                              
        
            # handle the possible blockage and collision situations   
            self.blockHandling(agents)    
            self.collisionHandling(agents)

            # increment time
            self.time += 1

        # once final solution is found, all paths are trimmed and appended to result list
        self.appendFinalPaths(agents, result)

        self.CPU_time = timer.time() - start_time
        
        # check whether there are collisions (debugging)        
        self.printCollisions(result)    
        # self.printResult(result)

        if self.time == 100 or self.CPU_time >time_limit:
                print(f"time limit hit in a map defined by: my_map {self.my_map}\n starts {self.starts}\n and goals {self.goals}")
                #raise Exception('TIME LIMIT')
                time_limit_reached = True
                # when the time limit is hit, return the results as an empty list to indicate that no solution was found 
                # do not penalize the cpu time fo the method for sitatuions where no results was found anyways
                result = []       
                self.CPU_time = 0

        return result, self.CPU_time
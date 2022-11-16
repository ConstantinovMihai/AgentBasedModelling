"""
This file contains the AircraftDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import copy

class AircraftDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    def __init__(self, my_map, start, goal, heuristics, agent_id):
        """
        my_map   - list of lists specifying obstacle positions
        starts      - (x1, y1) start location
        goals       - (x1, y1) goal location
        heuristics  - heuristic to goal location
        """

        self.my_map = my_map
        self.start = start
        self.goal = goal
        self.id = agent_id
        self.heuristics = heuristics
        self.current_heuristics = heuristics
        # list with the constraints for the agent
        self.constraints = []
        self.waiting = 0
        self.location = start
        # stores the path to goal
        self.path = []
        # stores the path the agent intends to take at any time iteration in the simulation
        self.planned_path = []
        # sense whether or not there is a blockage in its path
        self.blockage = False
        self


    def addConstraints(self, time, prox_loc):
        """ Add the constraints for an agent coordinating with other neighbouring agents 

        Args:
            time (int): the time at which the constraints are added
            prox_loc (list) : the locations at which other agents are (and have to be avoided)
        """

        '''
        # for each neighbouring agent, check if it reached its goal
        # also check if the current agent is blocked, if this is the case, no constraints should be added
        for neighbour in prox_loc:
            if self.blockage == False and neighbour['reached_goal'] == False:
                # if neighbour is not at goal and thus likely to move, implement its planned next n timesteps as constraints
                for t, constraint_loc in enumerate(neighbour['planned_path']):
                    self.constraints.append({'agent': self.id,'loc': [constraint_loc],'timestep': time+t+1, 'hard':False})
                # also add neighbours current location as a constraint, in the case that the neighbour will have to wait due to conflict resoltion
                # and thus will still occupy that cell in the next timestep
                self.constraints.append({'agent': self.id,'loc': [neighbour['location']],'timestep': time+1, 'hard':False})
            
            elif self.blockage == False and neighbour['reached_goal'] == True:
                # if neighbour has reached goal, the only reason it will move is if i am blocked.
                # Since i am not blocked, i should impose heavy penalty on that location to motivate myself to take another route
                for t in range (0,(25)):                      
                    self.constraints.append({'agent': self.id,'loc': [neighbour['planned_path'][0]],'timestep': time+t+1, 'hard':False})
        '''

        # reset the constraints list for this timestep
        self.constraints = []       
        # for each neigbour detected by the agent
        for neighbour in prox_loc:
            # if current agent's path to its goal is not blocked
            if self.blockage == False:
                # in time step 0 agents do not have a planned path yet
                if time > 0:
                    # append the neighbour's intended next location as a constraint
                    self.constraints.append({'agent': self.id,'loc': neighbour['planned_path'][0],'timestep': time+1, 'hard':False})
                # append the neighbours current location as a constraint in the next time step
                # this is for the case where the neighbour is forced to wait due to a conflict and cannot execute its planned path
                self.constraints.append({'agent': self.id,'loc': [neighbour['location']],'timestep': time+1, 'hard':False})
   



    
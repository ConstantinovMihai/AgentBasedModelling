"""
This file contains the AircraftDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

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
        # list with the constraints for the agent
        self.constraints = []
        self.location = start
        # stores the path to goal
        self.path = []

    def addBubbleConstraints(self, time, prox_loc):
        """ Add the bubble constraints for the agent (i.e. the locations of the neighbouring agents + a bubble around them) 

        Args:
            time (int): the time at which the constraints are added
            prox_loc (list) : the locations at which other agents are (and have to be avoided)
        """

        # iterate among each proximum agent
        self.constraints = []
        for neighbour in prox_loc:
            # the bubble constraints are only added, if the agent has not reached its goal and thus is likely to move in the next timestep, otherwise only the agents current (goal/final) location is stored as a constraint
            if neighbour['reached_goal'] == False:
                bubble =  [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
                # iterates among the bubble_locations (i.e. the places the agent might go in the next iteration)
                # TODO: accomodate for the sitaution when more than one move might be performed between two path computations
                for move in bubble:
                    constr_loc = neighbour['location'][0] + move[0], neighbour['location'][1] + move[1]
                    # add the constraint for the next x timesteps to motivate agent to take another path
                    for t in range (0,10):
                        self.constraints.append({'agent': self.id,'loc': [constr_loc],'timestep': time+t})
            else:
                constr_loc = neighbour['location'][0], neighbour['location'][1]
                for t in range (0,10):
                    self.constraints.append({'agent': self.id,'loc': [constr_loc],'timestep': time+t})



    
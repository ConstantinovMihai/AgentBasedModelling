import copy
import random

random.seed(421)

# this file contains the functions to generate the spawn and goal locations of the agents

def neighboursFilled(map : list, goal_locations : list, prop_location : tuple, y_spawn : list):
    """ computes the percentage of viable neighbours which are filled

    Returns:
       map : the map itself
       goal_locations (list) : the list of existing goals
       prop_locations (tuple) : the coordinates of the proposed location
       y_spawn (list) : y coordinates where agents can spawn 
    """
    tot_viable = 0
    viable_filled = 0

    for x in range(max(0, prop_location[0] - 1), min(8, prop_location[0] + 1) + 1):
        
        for y in y_spawn:
            if ((x,y) != prop_location):
                tot_viable += 1
                if ((x,y) in goal_locations):
                    viable_filled += 1

    return (viable_filled / tot_viable) * 100


def neighboursFilledGeneral(map : list, goal_locations : list, prop_location : tuple, y_spawn : list, perc_fill : float = 50):
    """ computes the percentage of viable neighbours which are filled

    Returns:
       map : the map itself
       goal_locations (list) : the list of existing goals
       prop_locations (tuple) : the coordinates of the proposed location
       y_spawn (list) : y coordinates where agents can spawn 
       perc_fill (float) : the maximum percentage of filled neighbouring cells allowed
    """
    
    goal_locations_aux = copy.copy(goal_locations)
    goal_locations_aux.append(prop_location)
    filled = False
    
    for loc in goal_locations_aux:
        perc_fill = neighboursFilled(map, goal_locations_aux, loc, y_spawn)
        if (perc_fill > 50):
            filled = True

    return filled



#function which generates the input file for simulation depending on the desired map, number if agents and spawn type
def createsSimulationInput(map_nb : int, nb_agents : int, spawn_type : int, perc_fill = 50):
    """ creates the simulation input based on the map, number of agents and the spawn type

    Args:
        map_nb (int): the map type number
        nb_agents (int): the number of agents to be created during the simulation
        spawn_type (int): index of the spawn type
        perc_fill (float) : the maximum percentage of filled neighbouring cells allowed
    Returns:
        list, list, list: the map with the obstacles, the list of start locations for the agents, the list of goal locations for the agents 
    """
    #the three types of maps which are of interest    
    if map_nb == 0:
        map = [[False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False]]
    elif map_nb == 1:
        map = [[False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, True, False, False, False, False, False, False, True, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, True, False, False, False, False, False, False, True, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, True, False, False, False, False, False, False, True, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, True, False, False, False, False, False, False, True, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False]]
    elif map_nb == 2:
        map = [[False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, True, False, False, False, False, False, False, True, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, True, False, True, True, True, True, False, True, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, True, False, False, False, False, False, False, True, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, True, False, False, False, False, False, False, True, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, True, False, True, True, True, True, False, True, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, True, False, False, False, False, False, False, True, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False]]

    start_locations = []
    goal_locations = []
    # a starting and goal location needs to be created for each agent
    while len(start_locations) < nb_agents:        
        #if agents all spawn on left side of map
        if spawn_type == 0:
            y_spawn = [20, 21]
            # select a random location in the two left most cells of the map
            start_location = (random.choice(range(0, 9)),random.choice([0,1]))   
            # if the location is already used generate new location             
            while start_location in start_locations:
                start_location = (random.choice(range(0, 9)),random.choice([0,1]))                
            start_locations.append(start_location)
            # find goal location at opposite side of map
            goal_location = (random.choice(range(0, 9)),random.choice(y_spawn)) 

            # percentage of viable neighboring cells already filled
            perc_filled_all = neighboursFilledGeneral(map, goal_locations, goal_location, y_spawn, perc_fill)
            
            while (goal_location in goal_locations or perc_filled_all == True):
                goal_location = (random.choice(range(0, 9)),random.choice(y_spawn))         
                # percentage of viable neighboring cells already filled
                perc_filled_all = neighboursFilledGeneral(map, goal_locations, goal_location, y_spawn, perc_fill) 
            
            goal_locations.append(goal_location)

        # if agents can spawn on either side of map
        elif spawn_type == 1:
            start_location = (random.choice(range(0, 9)),random.choice([0,1,20,21]))              
            while start_location in start_locations:
                start_location = (random.choice(range(0, 9)),random.choice([0,1,20,21]))               
            start_locations.append(start_location)

            y_spawn = [0, 1]
            if start_location[1] == 0 or start_location[1] == 1:
                y_spawn = [20, 21]

            # if agent spawned on left side of map, create goal location on opposite side        
            goal_location = (random.choice(range(0, 9)),random.choice(y_spawn))

            # percentage of viable neighboring cells already filled
            perc_filled_all = neighboursFilledGeneral(map, goal_locations, goal_location, y_spawn, perc_fill)
            while (goal_location in goal_locations or perc_filled_all == True):
                goal_location = (random.choice(range(0, 9)),random.choice(y_spawn))         
                # percentage of viable neighboring cells already filled
                perc_filled_all = neighboursFilledGeneral(map, goal_locations, goal_location, y_spawn, perc_fill) 

            goal_locations.append(goal_location)

    assert(len(start_locations) == len(goal_locations))
    return map, start_locations, goal_locations








            


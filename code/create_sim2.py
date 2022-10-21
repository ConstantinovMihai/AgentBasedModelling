import random


random.seed(421)
#function which generates the input file for simulation depending on the desired map, number if agents and spawn type
def createsSimulationInput(map_nb,nb_agents,spawn_type):
    """ creates the simulation input based on the map, number of agents and the spawn type

    Args:
        map_nb (int): the map type number
        nb_agents (int): the number of agents to be created during the simulation
        spawn_type (int): index of the spawn type

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
            # select a random location in the two left most cells of the map
            start_location = (random.choice(range(0, 9)),random.choice([0,1]))   
            # if the location is already used generate new location             
            while start_location in start_locations:
                start_location = (random.choice(range(0, 9)),random.choice([0,1]))                
            start_locations.append(start_location)
            # find goal location at opposite side of map
            goal_location = (random.choice(range(0, 9)),random.choice([20,21]))              
            while goal_location in goal_locations:
                goal_location = (random.choice(range(0, 9)),random.choice([20,21]))          
            goal_locations.append(goal_location)
        # if agents can spawn on either side of map
        elif spawn_type == 1:
            start_location = (random.choice(range(0, 9)),random.choice([0,1,20,21]))              
            while start_location in start_locations:
                start_location = (random.choice(range(0, 9)),random.choice([0,1,20,21]))               
            start_locations.append(start_location)

            # if agent spawned on left side of map, create goal location on opposite side
            if start_location[1] == 0 or start_location[1] == 1:
                goal_location = (random.choice(range(0, 9)),random.choice([20,21]))
                while goal_location in goal_locations:
                    goal_location = (random.choice(range(0, 9)),random.choice([20,21]))
            # if agent spawned on right side of map, create goal location on opposite side
            if start_location[1] == 20 or start_location[1] == 21:
                goal_location = (random.choice(range(0, 9)),random.choice([0,1]))
                while goal_location in goal_locations:
                    goal_location = (random.choice(range(0, 9)),random.choice([0,1]))
            goal_locations.append(goal_location)
    
    assert(len(start_locations) == len(goal_locations))
    return map, start_locations, goal_locations








            


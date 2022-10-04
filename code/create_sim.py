import random


random.seed(421)
#function which generates the input file for simulation depending on the desired map, number if agents and spawn type
def generatesSimulation(map_nb,nb_agents,spawn_type):
    #the three types of maps which are of interest    
    if map_nb == 0:
        map = """9 21
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .
. . . . . . . . . . . . . . . . . . . . . .
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .
. . . . . . . . . . . . . . . . . . . . . .
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .
. . . . . . . . . . . . . . . . . . . . . .
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .
. . . . . . . . . . . . . . . . . . . . . .
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . ."""
    elif map_nb == 1:
        map = """9 21
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .
. . . . . . . @ . . . . . . @ . . . . . . .
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .
. . . . . . . @ . . . . . . @ . . . . . . .
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .
. . . . . . . @ . . . . . . @ . . . . . . .
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .
. . . . . . . @ . . . . . . @ . . . . . . .
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . ."""
    elif map_nb == 2:
        map = """9 21
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .
. . . . . . . @ . . . . . . @ . . . . . . .
. . @ @ @ @ . @ . @ @ @ @ . @ . @ @ @ @ . .
. . . . . . . @ . . . . . . @ . . . . . . .
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . .
. . . . . . . @ . . . . . . @ . . . . . . .
. . @ @ @ @ . @ . @ @ @ @ . @ . @ @ @ @ . .
. . . . . . . @ . . . . . . @ . . . . . . .
. . @ @ @ @ . . . @ @ @ @ . . . @ @ @ @ . ."""
    
    locations = []
    start_locations = []
    goal_locations = []
    # a starting and goal location needs to be created for each agent
    while len(locations) < nb_agents:        
        #if agents all spawn on left side of map
        if spawn_type == 0:
            # select a random location in the two left most cells of the map
            start_location = [random.choice([0,1]),random.choice(range(0, 9))]   
            # if the location is already used generate new location             
            while start_location in start_locations:
                start_location = [random.choice([0,1]),random.choice(range(0, 9))]                
            start_locations.append(start_location)
            # find goal location at opposite side of map
            goal_location = [random.choice([20,21]),random.choice(range(0, 9))]                
            while goal_location in goal_locations:
                goal_location = [random.choice([20,21]),random.choice(range(0, 9))]                
            goal_locations.append(goal_location)
        # if agents can spawn on either side of map
        elif spawn_type == 1:
            start_location = [random.choice([0,1]),random.choice(range(0, 9))]                
            while start_location in start_locations:
                start_location = [random.choice([0,1]),random.choice(range(0, 9))]                
            start_locations.append(start_location)

            # if agent spawned on left side of map, create goal location on opposite side
            if start_location[0] == 0 or start_location[0] == 1:
                goal_location = [random.choice([20,21]),random.choice(range(0, 9))]
                while goal_location in goal_locations:
                    goal_location = [random.choice([20,21]),random.choice(range(0, 9))]
            # if agent spawned on right side of map, create goal location on opposite side
            if start_location[0] == 20 or start_location[0] == 21:
                goal_location = [random.choice([0,1]),random.choice(range(0, 9))]
                while goal_location in goal_locations:
                    goal_location = [random.choice([20,21]),random.choice(range(0, 9))]
            goal_locations.append(goal_location)
        locations.append([start_location[1],start_location[0],goal_location[1],goal_location[0]])
    
    # write the results in a file
    filename = f"experimental_setup/map_{map_nb}-agents_{nb_agents}-type_{spawn_type}.txt"
    with open(filename, 'w') as f:
        f.write(map+"\n")
        f.write(str(nb_agents)+"\n")
        for i in locations:
            f.write(f"{i[0]} {i[1]} {i[2]} {i[3]}"+"\n")
    return






            


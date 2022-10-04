import random


random.seed(421)

def generatesSimulation(map_nb,nb_agents,spawn_type, idx):
    
    #TODO: DOCUMENT THIS FUNCTION
    """_summary_

    Args:
        map_nb (_type_): _description_
        nb_agents (_type_): _description_
        spawn_type (_type_): _description_
        idx (int): index of the map from this instance (to avoid overwriting)
    """    
    
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
    while len(locations) < nb_agents:        
        
        if spawn_type == 0:
            start_location = [random.choice([0,1]),random.choice(range(0, 9))]                
            while start_location in start_locations:
                start_location = [random.choice([0,1]),random.choice(range(0, 9))]                
            start_locations.append(start_location)
                
            goal_location = [random.choice([20,21]),random.choice(range(0, 9))]  
                   
            while goal_location in goal_locations:
                goal_location = [random.choice([20,21]),random.choice(range(0, 9))]                
            goal_locations.append(goal_location)
            
        elif spawn_type == 1:
            start_location = [random.choice([0,1]),random.choice(range(0, 9))]                
            while start_location in start_locations:
                start_location = [random.choice([0,1]),random.choice(range(0, 9))]                
            start_locations.append(start_location)
           
            if start_location[0] == 0 or start_location[0] == 1:
                goal_location = [random.choice([20,21]),random.choice(range(0, 9))]
                while goal_location in goal_locations:
                    goal_location = [random.choice([20,21]),random.choice(range(0, 9))]

            if start_location[0] == 20 or start_location[0] == 21:
                goal_location = [random.choice([0,1]),random.choice(range(0, 9))]
                while goal_location in goal_locations:
                    goal_location = [random.choice([20,21]),random.choice(range(0, 9))]
            goal_locations.append(goal_location)
        locations.append([start_location[1],start_location[0],goal_location[1],goal_location[0]])
    
    # write the results in a file
    filename = f"experimental_setup/map_{map_nb}-agents_{nb_agents}-type_{spawn_type}-idx_{idx}.txt"
    with open(filename, 'w') as f:
        f.write(map+"\n")
        f.write(str(nb_agents)+"\n")
        for i in locations:
            f.write(f"{i[0]} {i[1]} {i[2]} {i[3]}"+"\n")






            


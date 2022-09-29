import random
def generate_sim(map_nb,nb_agents,spawn_type):
    
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
    while len(locations) < nb_agents:        
        start_locations = []
        goal_locations = []
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
            if start_location[0] == 0 or goal_location[0] == 1:
                goal_location = [random.choice([20,21]),random.choice(range(0, 9))]
                while goal_location in goal_locations:
                    goal_location = [random.choice([20,21]),random.choice(range(0, 9))]
            if start_location[0] == 20 or goal_location[0] == 21:
                goal_location = [random.choice([0,1]),random.choice(range(0, 9))]
                while goal_location in goal_locations:
                    goal_location = [random.choice([20,21]),random.choice(range(0, 9))]
                goal_locations.append(goal_location)
        locations.append([start_location[1],start_location[0],goal_location[1],goal_location[0]])
        

    #text = 
    print(*locations, sep = "\n")
    filename = f"{map_nb}{nb_agents}{spawn_type}.txt"
    with open(filename, 'w') as f:
        f.write(map+"\n")
        f.write(str(nb_agents)+"\n")
        for i in locations:
            f.write(f"{i[0]} {i[1]} {i[2]} {i[3]}"+"\n")


    return locations

print(generate_sim(0,6,0))


        

            


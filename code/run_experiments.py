"""
Main file to run experiments and show animation.

Note: To make the animation work in Spyder you should set graphics backend to 'Automatic' (Preferences > Graphics > Graphics Backend).
"""

#!/usr/bin/python
import argparse
from cProfile import run
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from distributed import DistributedPlanningSolver # Placeholder for Distributed Planning
from visualize import Animation
from single_agent_planner import get_sum_of_cost
from create_sim2 import generatesSimulation
import numpy as np
import matplotlib.pyplot as plt
import pickle

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    """
    Prints start location and goal location of all agents, using @ for an obstacle, . for a open cell, and 
    a number for the start location of each agent.
    
    Example:
        @ @ @ @ @ @ @ 
        @ 0 1 . . . @ 
        @ @ @ . @ @ @ 
        @ @ @ @ @ @ @ 
    """
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)

def print_locations(my_map, locations):
    """
    See docstring print_mapf_instance function above.
    """
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)

def import_mapf_instance(filename):
    """
    Imports mapf instance from instances folder. Expects input as a .txt file in the following format:
        Line1: #rows #columns (number of rows and columns)
        Line2-X: Grid of @ and . symbols with format #rows * #columns. The @ indicates an obstacle, whereas . indicates free cell.
        Line X: #agents (number of agents)
        Line X+1: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 1)
        Line X+2: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 2)
        Line X+n: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent n)
        
    Example:
        4 7             # grid with 4 rows and 7 columns
        @ @ @ @ @ @ @   # example row with obstacle in every column
        @ . . . . . @   # example row with 5 free cells in the middle
        @ @ @ . @ @ @
        @ @ @ @ @ @ @
        2               # 2 agents in this experiment
        1 1 1 5         # agent 1 starts at (1,1) and has (1,5) as goal
        1 2 1 4         # agent 2 starts at (1,2) and has (1,4) as goal
    """
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()

    return my_map, starts, goals


def processArgs(args, my_map, starts, goals ):

    time = 0
    paths = []

    if args.solver == "CBS":
        print("***Run CBS***")
        cbs = CBSSolver(my_map, starts, goals)
        paths, time = cbs.find_solution(args.disjoint)
    elif args.solver == "Independent":
        print("***Run Independent***")
        solver = IndependentSolver(my_map, starts, goals)
        paths, time = solver.find_solution()
    elif args.solver == "Prioritized":
        print("***Run Prioritized***")
        solver = PrioritizedPlanningSolver(my_map, starts, goals)
        paths, time = solver.find_solution()
    elif args.solver == "Distributed":  # Wrapper of distributed planning solver class
        print("***Run Distributed Planning***")
        solver = DistributedPlanningSolver(my_map, starts, goals, ...) #!!!TODO: add your own distributed planning implementation here.
        paths, time = solver.find_solution()
    else: 
        raise RuntimeError("Unknown solver!")

    return paths, time


def runSimulation(args, results, saveCVS = False): # NO LONGER USED
    """Runs the simulation using the provided args and either stores the data or puts in the results dictionary 

    Args:
        args (list): the arguments from the command line
        results (list, optional): stores the total cost and the time needed to run a particular experiment. Defaults to {}.
        saveCVS (bool, optional): if set to true it will write the results in a CVS file. Defaults to False.
    """
    result_file = open("results.csv", "w", buffering=1)

    # iterates among all the files 
    for file in sorted(glob.glob(args.instance)):

        print("***Import an instance***")
        #my_map, starts, goals = import_mapf_instance(file)
        my_map, starts, goals = generatesSimulation(0,3,0)
        print_mapf_instance(my_map, starts, goals)

        # get paths and time for the simulation
        paths, time = processArgs(args, my_map, starts, goals )

        # computes the total cost
        cost = get_sum_of_cost(paths)

        # if saveCVS is set to true write in the a CVS file
        if saveCVS:
            result_file.write("{},{},{}\n".format(file, cost, round(time, 6)))
        

        # process the file key to get file_key: first element contains map-agent-type, the second elements contains the index
        file_key = ((file.split("/")[1]).split(".")[0]).split("-idx_")
        
        # check if the key is already in the dict, if not, create it
        if file_key[0] not in results:
            results[file_key[0]] = np.array((cost, round(time, 6)))
        # add the simulation results to the corresponding dict
        else:
            results[file_key[0]] = np.append(results[file_key[0]], (cost, round(time, 6)))

        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
            animation.show()

    result_file.close()
    

def testPathSimulation(args, my_map, starts, goals, paths):
    if not args.batch:
        print("***Test paths on a simulation***")
        animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
        animation.show() 


def runOneExperiment(map, agent, spawn_type, results):
    """ Runs on experiment for a certain instance

    Args:
        map (_type_): _description_
        agent (_type_): _description_
        spawn_type (_type_): _description_
        results (_type_): _description_
    """
    my_map, starts, goals = generatesSimulation(map, agent, spawn_type)
                    
    # get paths and time for the simulation
    paths, time = processArgs(args, my_map, starts, goals )

    # computes the total cost
    cost = get_sum_of_cost(paths)

    # process the file key to get file_key: first element contains map-agent-type, the second elements contains the index
    file_key = f"map_{map}-agent_{agent}-spawn_{spawn_type}"
    # check if the key is already in the dict, if not, create it
    if file_key not in results:
        results[file_key] = np.array((cost, round(time, 6)))
    # add the simulation results to the corresponding dict
    else:
        results[file_key] = np.append(results[file_key], (cost, round(time, 6)))

    testPathSimulation(args, my_map, starts, goals, paths)


def generateExperiments(nb_maps, max_agents, nb_spawns, results, args):
    """_summary_

    Args:
        nb_maps (_type_): _description_
        max_agents (_type_): _description_
        nb_spawns (_type_): _description_
        results (_type_): description
        args (_type_): _description_
    """
    for map in range(nb_maps):
        for agent in range(2, max_agents):
            for spawn_type in range(nb_spawns):
                # TODO: IMPLEMENT THE STATISTICAL METHODS HERE
                key = f"map_{map}-agent_{agent}-spawn_{spawn_type}"
                variation_cost = [0]
                variation_time = [0]

                # condition for stopping to iterate:
                # if the current variation coefficient is within a half of a standard deviation from
                # the mean of the last quarter of the simulations
                while (len(variation_time) < 50 or (np.std(variation_time[-30:]) < 0.5 * np.mean(variation_time[-30:]))
                    or (np.std(variation_cost[-30:]) < 0.5 * np.mean(variation_cost[-30:]))):
                    print(f"len(variation_time) IS {len(variation_time)}")
                    runOneExperiment(map, agent, spawn_type, results)
                    data = results[key]
                    variation_cost = data[::2]
                    variation_time = data[1::2]
                    # the variation coefficient = standard deviation / mean
                    variation_cost = np.append(variation_cost, np.std(data[::2]) / np.mean(data[::2]))
                    variation_time = np.append(variation_time, np.std(data[1::2]) / np.mean(data[1::2]))
                    if (len(variation_time) == 150):
                        break

                print(f"len of variation is {len( variation_time)}")
                x = np.arange(len(variation_time))
                plt.plot(x, variation_time)
                plt.xlabel("iteration")
                plt.ylabel("coefficient of variation for time")
                plt.show()




def parseArgs():
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()
    return args


def processResults(results):
    for key in results.keys():
        data = results[key]

        # python slicing: start - stop -step
        costs = data[::2]
      
        times = data[1::2]
        plotData(times)


def plotData(data):
    """ Plotting routine

    Args:
        data (_type_): _description_
    """
    #TODO: Improve

    plt.hist(data)
    plt.show()


def runSimulation(args):
    """ Runs the experiments and saves the results in a pickle structure
    """

    generateExperiments(nb_maps=2, max_agents=3, nb_spawns=1, results=results, args=args)
    # save the dicionary
    with open('saved_dictionary.pkl', 'wb') as f:
        pickle.dump(results, f)


if __name__ == '__main__':

    # generateExperiments(nb_maps=2, max_agents=3, nb_spawns=1)
    
    args = parseArgs()
    results = {}

    runSimulation(args)
    
    # load the dictionary with the results
    with open('saved_dictionary.pkl', 'rb') as f:
        results = pickle.load(f)

    
    # print(results.keys())

    processResults(results)

    

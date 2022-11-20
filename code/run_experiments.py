"""
Main file to run experiments and show animation.

Note: To make the animation work in Spyder you should set graphics backend to 'Automatic' (Preferences > Graphics > Graphics Backend).
"""

#!/usr/bin/python
import argparse
from cProfile import run
import glob
from distributed_individual import DistributedPlanningSolverIndividual # Placeholder for Distributed Planning
from visualize import Animation
from single_agent_planner import getSumOfCost
from create_sim2 import createsSimulationInput
import numpy as np
import matplotlib.pyplot as plt
import pickle
import utilities
import time as timer
from datetime import datetime

SOLVER = "CBS"

def testPathSimulation(args, my_map, starts, goals, paths, animate = False):
  

    if not args.batch and animate:
        print("***Test paths on a simulation***")
        animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
        animation.show() 


def runOneExperiment(map, nb_agents, spawn_type, results, animate=False, perc_fill = 50):
    """ Runs on experiment for a certain instance (i.e a certain map-nb_agents-spawn_type combination)

    Args:
        map (int): maximum number of maps in all simulations
        nb_agents (int): maximum number of agents in all the simulations
        spawn_type (int): 0 is for agent moving in one way, 2 for agents with spawns in both ends of the map
       results (dict): stores the results for the iterations for each experimental instance
    """
    my_map, starts, goals = createsSimulationInput(map, nb_agents, spawn_type, perc_fill)
    print('map')
                    
    # get paths and time for the simulation
    paths, time = utilities.processArgs(args, my_map, starts, goals )

    # computes the weighted cost per agent
    cost = getSumOfCost(paths) / nb_agents

    # process the file key to get file_key: first element contains map-agent-type, the second elements contains the index
    file_key = f"map_{map}-agent_{nb_agents}-spawn_{spawn_type}"
    # check if the key is already in the dict, if not, create it
    if file_key not in results:
        results[file_key] = np.array((cost,time))
    # add the simulation results to the corresponding dict
    else:
        results[file_key] = np.append(results[file_key], (cost, time))

    testPathSimulation(args, my_map, starts, goals, paths, animate)



def generateExperiments(nb_maps, max_agents, nb_spawns, results, min_agents, min_map, animate, perc_fill, plotVar):
    """ Generates experiments for all possible combinations of maps, nb of agents and spawn types until
        the coefficient of variation stabilises  

    Args:
        nb_maps (int): maximum number of maps in all simulations
        max_agents (int): maximum number of agents in all the simulations
        nb_spawns (int): maximum number of spawn types (usually, there will be only 2 options)
        results (dict): stores the results for the iterations for each experimental instance
        args (string): command line arguments
        min_agents (int) : minimum number of agents in all the simulations
        min_map (int) : minimum number of maps in all simulations (for debugging purposes)
        perc_fill (float) : the maximum percentage of filled neighbouring cells allowed
    """

    # iterates among all the maps, agents and spawn types
    for map in range(min_map, nb_maps + 1):
        for agent in range(min_agents, max_agents + 1):
            for spawn_type in nb_spawns:
                # name of the key in the results dict
                key = f"map_{map}-agent_{agent}-spawn_{spawn_type}"
                now = datetime.now()

                current_time = now.strftime("%H:%M:%S")
                print(key)
                print(args.solver)
                print("Current Time =", current_time)

                # avoids "referenced before assignment error"
                variation_cost = [0]
                variation_time = [0]

                # condition for stopping to iterate:
                # if the current variation coefficient is within a half of a standard deviation from
                # the mean of the last quarter of the simulations
                counter = 0
                while (np.std(variation_cost[-100:]) >= 0.25 * np.mean(variation_cost[-100:])) and counter < 250:
                    counter += 1
                    now = datetime.now()
                    current_time = now.strftime("%H:%M:%S")
                    # runs one experiment
                    print("Current Time =", current_time)
                    runOneExperiment(map, agent, spawn_type, results, animate, perc_fill)
                    
                    data = results[key]
                  

                    # find the indeces where cost is not 0 (the valid experiments)
                    # store their indeces
                    valid_experiments = np.nonzero(data[0::2])
                   
                    failed_exp_perc = 1 - np.count_nonzero(data[0::2]) / len(data[0::2])

                    # store the costs and times obtained in the valid experiments
                    valid_costs = data[::2][valid_experiments]
                    valid_times = data[1::2][valid_experiments]

                    # the variation coefficient = standard deviation / mean
                    # those are to be computed for the entire data set
                    variation_cost = np.append(variation_cost, np.std(valid_costs) / np.mean(valid_costs))
                    variation_time = np.append(variation_time, np.std(valid_times) / np.mean((valid_times)))
                   
                
                failed_exp_perc = 1 - np.count_nonzero(results[key][0::2]) / len(results[key][0::2])
                results[key] = {"failed_perc" : failed_exp_perc, "mean_cost" : np.mean(valid_costs), "mean_time" : np.mean(valid_times),
                "std_cost" : np.std(valid_costs), "std_time" : np.std(valid_times)}
                

                if plotVar:
                    # print(f"valid_costs {valid_costs}")
                    # print(f"valid_times {valid_times}")
                    plotVariation(variation_time, valid_times, variation_cost)


def plotVariation(variation_time : float, valid_times : list, variation_cost : float):
    """ Generates the stabilisation of coefficient of variation plot

    Args:
        variation_time (float): coefficient of variation for the times variable of the experiments
        valid_times (list): the CPU times for the experiments with a valid solution
        variation_cost (float): coefficient of variation for the costs variable of the experiments
    """
    
    x = np.arange(len(variation_time))

    np.savetxt('validtimes.txt', valid_times, delimiter=',')
    np.savetxt('variation_time.txt', variation_time, delimiter=',')

    # plot of the costs&times
    plt.plot(x, variation_cost, color="red", label="cost")
    plt.plot(x, variation_time, color="green", label="time")
    plt.ylim(bottom=0)
    plt.legend()
    plt.show()


def runSimulation(args, animate=False, perc_fill = 65, nb_maps=3, max_agents=10, nb_spawns=[1], min_agents=8, min_map=2, plotVar=True):
    """ Runs the experiments and saves the results in a pickle structure
    Args:
        args (str) : string with the arguments passed through the terminal by the user
        animeate (False) : when set True, the animation of the agents will be showed
        perc_fill (float) : the maximum percentage of filled neighbouring cells allowed
        plotVar (bool) : plots the plot of the stabilisation of coefficient of variation when set True
    """

    results = {}
    generateExperiments(nb_maps, max_agents, nb_spawns, results, min_agents, min_map, animate, perc_fill, plotVar)
    
    save_data = [results, min_agents, max_agents, min_map, nb_maps, nb_spawns]
    # save the dicionary containing all the results
    with open(f'saved_dictionary_{args.solver}.pkl', 'wb') as f:
        pickle.dump(save_data, f)


def testExistingMaps(args):
    """ This function runs the algos on existings maps
    map (int): maximum number of maps in all simulations
    agents (int): maximum number of agents in all the simulations
    spawn (int): maximum number of spawn types (usually, there will be only 2 options)
    results (dict): stores the results for the iterations for each experimental instance
    """
    
    
    # starts = [(0, 1), (3, 1), (7, 0), (5, 0), (4, 0), (4, 1), (2, 0)]
    # goals = [(5, 21), (8, 20), (6, 21), (7, 20), (8, 21), (0, 20), (6, 20)]
    # my_map = [[False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False]]
    my_map, starts, goals = utilities.import_mapf_instance('dist_test.txt')
    paths, time = utilities.processArgs(args, my_map, starts, goals)
    testPathSimulation(args, my_map, starts, goals, paths, animate=True)


if __name__ == '__main__':
        
    args = utilities.parseArgs()
  
    # testExistingMaps(args)
    runSimulation(args, animate=False, perc_fill = 75, nb_maps=2, max_agents=10, nb_spawns=[0,1], min_agents=1, min_map=0, plotVar=False)

    results = {}

    # load the dictionary with the results
    # with open('saved_dictionary.pkl', 'rb') as f:
    #     results = pickle.load(f)



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

SOLVER = "CBS"

def testPathSimulation(args, my_map, starts, goals, paths, animate = False):
    if not args.batch and animate:
        print("***Test paths on a simulation***")
        animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
        animation.show() 


def runOneExperiment(map, agent, spawn_type, results, animate=False):
    """ Runs on experiment for a certain instance (i.e a certain map-nb_agents-spawn_type combination)

    Args:
        map (_type_): _description_
        agent (_type_): _description_
        spawn_type (_type_): _description_
        results (_type_): _description_
    """
    my_map, starts, goals = createsSimulationInput(map, agent, spawn_type)
                    
    # get paths and time for the simulation
    paths, time = utilities.processArgs(args, my_map, starts, goals )

    # computes the total cost
    cost = getSumOfCost(paths)

    # process the file key to get file_key: first element contains map-agent-type, the second elements contains the index
    file_key = f"map_{map}-agent_{agent}-spawn_{spawn_type}"
    # check if the key is already in the dict, if not, create it
    if file_key not in results:
        results[file_key] = np.array((cost, round(time, 6)))
    # add the simulation results to the corresponding dict
    else:
        results[file_key] = np.append(results[file_key], (cost, round(time, 6)))

    testPathSimulation(args, my_map, starts, goals, paths, animate)


def generateExperiments(nb_maps, max_agents, nb_spawns, results, args, min_agents = 2, min_map = 0, animate = False):
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
    """

    # iterates among all the maps, agents and spawn types
    for map in range(min_map, nb_maps):
        for agent in range(min_agents, max_agents+1):
            for spawn_type in range(nb_spawns):
                # name of the key in the results dict
                key = f"map_{map}-agent_{agent}-spawn_{spawn_type}"

                # avoids "referenced before assignment error"
                variation_cost = [0]
                variation_time = [0]

                # condition for stopping to iterate:
                # if the current variation coefficient is within a half of a standard deviation from
                # the mean of the last quarter of the simulations
                while ((np.std(variation_time[-100:]) >= 0.25 * np.mean(variation_time[-100:]))
                    or (np.std(variation_cost[-100:]) >= 0.25 * np.mean(variation_cost[-100:]))):
                    
                    # runs one experiment
                    runOneExperiment(map, agent, spawn_type, results, animate)
                    
                    data = results[key]

                    # find the indeces where cost is 0 (the valid experiments)
                    # store their indeces
                    valid_experiments = np.nonzero(data[::2])

                    # store the costs and times obtained in the valid experiments
                    valid_costs = data[::2][valid_experiments]
                    valid_times = data[1::2][valid_experiments]

                    # the variation coefficient = standard deviation / mean
                    # those are to be computed for the entire data set
                    variation_cost = np.append(variation_cost, np.std(valid_costs) / np.mean(valid_costs))
                    variation_time = np.append(variation_time, np.std(valid_times) / np.mean((valid_times)))
                    #TODO: remove this later when we re sure the implementation is not stucked 
                    if (len(variation_time) == 200):
                        break
                
                    # TODO: remove this later when we're sure this routine works as intended
                    print(f"len of variation is {len( variation_time)}")
                x = np.arange(len(variation_time))

                np.savetxt('validtimes.txt', valid_times, delimiter=',')
                np.savetxt('variation_time.txt', variation_time, delimiter=',')

                # plot of the costs&times
                plt.plot(x, variation_cost, color="red", label="cost")
                plt.plot(x, variation_time, color="green", label="time")
                plt.ylim(bottom=0)
                plt.legend()
                plt.show()


def runSimulation(args, animate=False):
    """ Runs the experiments and saves the results in a pickle structure
    """

    generateExperiments(nb_maps=2, max_agents=3, nb_spawns=1, min_agents=3, args=args, results=results, animate=animate)
    # save the dicionary
    with open('saved_dictionary.pkl', 'wb') as f:
        pickle.dump(results, f)


def testExistingMaps(args):
    """ This function runs the algos on existings maps
    map (int): maximum number of maps in all simulations
    agents (int): maximum number of agents in all the simulations
    spawn (int): maximum number of spawn types (usually, there will be only 2 options)
    results (dict): stores the results for the iterations for each experimental instance
    """
    
    # my_map = [[False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False]]
    # my_map = [[False, False, False],[False, False, False],[False, False, False],[False, False, False]]
    starts = starts = [(1, 1), (2, 0), (7, 1)]
    goals =  goals = [(1, 20), (0, 20), (2, 21)]
    my_map = [[False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False], [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False], [False, False, True, True, True, True, False, False, False, True, True, True, True, False, False, False, True, True, True, True, False, False]]
    # my_map, starts, goals = utilities.import_mapf_instance('dist_test.txt')
    paths, time = utilities.processArgs(args, my_map, starts, goals)
    testPathSimulation(args, my_map, starts, goals, paths, animate=True)
    pass


if __name__ == '__main__':
        
    args = utilities.parseArgs()
    results = {}

    testExistingMaps(args)

    # runSimulation(args, animate=False)
    
    # load the dictionary with the results
    with open('saved_dictionary.pkl', 'rb') as f:
        results = pickle.load(f)


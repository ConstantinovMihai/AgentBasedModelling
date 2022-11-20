"""
The datasets are statistically analysed in this file
"""

import numpy as np
import statistical_tests
import pickle
import matplotlib.pyplot as plt
import re

if __name__ == '__main__':
    results = {}
    with open('saved_dictionary_Prioritized.pkl', 'rb') as f:
        results = pickle.load(f)[0]

    # stores the simulation time
    times = np.zeros(11)
    failures = np.zeros(11)

    # total number of agents
    nb_agents_arr = np.arange(1, 11)
    
    costs_arr = []

    for exp, exp_results in results.items():
        # contains map, nb agents, spawn type
        
        nb_agents = int(re.findall(r'\d+', exp.split("-")[1])[0])
        
        print(f"exp {exp_results}")
        times[nb_agents] += exp_results['mean_time'] / 6
        failures[nb_agents] += exp_results['failed_perc'] / 6
        
        costs_arr.append(exp_results['mean_cost'])

    print(costs_arr)
    plt.plot(nb_agents_arr, times[1:])
    plt.xlabel("Nb of agents")
    plt.ylabel("CPU time")
    plt.grid()
    plt.show()
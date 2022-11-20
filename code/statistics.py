"""
The datasets are statistically analysed in this file
"""

import numpy as np
import statistical_tests as stt
import pickle
import matplotlib.pyplot as plt
import re


def getTimeFailuresArray(filename):
    results = {}
    with open(filename, 'rb') as f:
        results = pickle.load(f)[0]

    # stores the simulation time
    times = np.zeros(11)
    failures = np.zeros(11)

    # total number of agents
    nb_agents_arr = np.arange(1, 11)
    nb_valid_exp = np.zeros(11)

    costs_arr = []

    for exp, exp_results in results.items():
        # contains map, nb agents, spawn type
        
        nb_agents = int(re.findall(r'\d+', exp.split("-")[1])[0])
        if not np.isnan(exp_results['mean_time']):
            nb_valid_exp[nb_agents] += 1
            times[nb_agents] += exp_results['mean_time']
            failures[nb_agents] += exp_results['failed_perc']
            costs_arr.append(exp_results['mean_cost'])

    return times[1:] / nb_valid_exp[1:], failures[1:] / nb_valid_exp[1:] * 100, nb_agents_arr, costs_arr


if __name__ == '__main__':

    results = {}
    timesCBS, failuresCBS, nb_agents_arr, costsCBS = getTimeFailuresArray("saved_dictionary_CBS.pkl")
    timesPrioritized, failuresPriotized, _, costPrioritized = getTimeFailuresArray("saved_dictionary_Prioritized.pkl")
    timesDistributed, failuresDistibuted, _, costDistributed = getTimeFailuresArray("saved_dictionary_Distributed.pkl")

    # print(costsCBS)
    print(np.mean(costsCBS))
    print(np.mean(costPrioritized))
    # print(costPrioritized)
    print(f"test between CBS and A* {stt.statisticalTests(costsCBS, costPrioritized, significance_lvl=0.05)}")

    # plt.plot(nb_agents_arr, timesCBS, label="CBS")
    # plt.plot(nb_agents_arr, timesPrioritized, label="Prioritized")
    # plt.plot(nb_agents_arr, timesDistributed, label="Distributed")
    # print(failuresCBS)

    plt.plot(nb_agents_arr, failuresCBS, label="CBS")
    plt.plot(nb_agents_arr, failuresPriotized, label="Prioritized")
    plt.plot(nb_agents_arr, failuresDistibuted, label="Distributed")
    plt.legend()
    plt.xlabel("Nb of agents")
    plt.ylabel("Percentage of failures (%)")
    plt.grid()
    plt.show()
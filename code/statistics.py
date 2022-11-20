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
        spawn_type = int(re.findall(r'\d+', exp.split("-")[2])[0])
        

        if not np.isnan(exp_results['mean_time']):
            nb_valid_exp[nb_agents] += 1
            times[nb_agents] += exp_results['mean_time']
            failures[nb_agents] += exp_results['failed_perc']
            costs_arr.append(exp_results['mean_cost'])

    return times[1:] / nb_valid_exp[1:], failures[1:] / nb_valid_exp[1:] * 100, nb_agents_arr, costs_arr


def plotPerformanceIndicator(x_axis, y1, y2, y3, labels, y_label):
    """ Plots the time graph

    Args:
        x_axis (list): x axis values
        times (list): time values
        label (str): the label of the graph 
        y_label (str): label for the graph
    """

    plt.plot(nb_agents_arr, y1, label=labels[0])
    plt.plot(nb_agents_arr, y2, label=labels[1])
    plt.plot(nb_agents_arr, y3, label=labels[2])
    plt.legend()
    plt.xlabel("Nb of agents")
    plt.ylabel(y_label)
    plt.grid()
    plt.show()


if __name__ == '__main__':

    results = {}
    timesCBS, failuresCBS, nb_agents_arr, costsCBS = getTimeFailuresArray("saved_dictionary_CBS.pkl")
    timesPrioritized, failuresPriotized, _, costPrioritized = getTimeFailuresArray("saved_dictionary_Prioritized.pkl")
    timesDistributed, failuresDistibuted, _, costDistributed = getTimeFailuresArray("saved_dictionary_Distributed.pkl")

    # uncomment to get the statistical
    # # print(costsCBS)
    # print(np.mean(costDistributed))
    # print(np.mean(costPrioritized))
    #print(f"test between CBS and A* {stt.statisticalTests(costsCBS, costPrioritized, significance_lvl=0.05)}")

    plotPerformanceIndicator(nb_agents_arr, failuresCBS, failuresDistibuted, failuresPriotized, labels=["CBS", "Distributed", "Prioritized"], y_label=f"Percentage of failures ($\%$)")
    plotPerformanceIndicator(nb_agents_arr, timesCBS, timesDistributed, timesPrioritized, labels=["CBS", "Distributed", "Prioritized"], y_label="CPU time (s)")

    print("done")
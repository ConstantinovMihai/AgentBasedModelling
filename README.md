# AgentBasedModelling
Multiagent Path Planning Assignment

This project contains the implementation of three planning techniques:
1. Prioritized planning - see prioritized.py
2. Conflict Based Search - see cbs.py
3. Distributed planner - see distributed_class.py and distributed_individual.py

Also, a dummy solver can be found in independent.py. Single_agent_planner.py and utilities.py contain utility function for the aforementioned files. Aircraft.py contains the definition of the class agent.

To run the model, run from the command line
python run_experiments.py  --solver SolverName
Where SolverName might be Prioritized, CBS or Distributed, e.g.
python run_experiments.py  --solver Distributed
would run distributed

This will create the maps and run them until the coefficient of variation stabilises.
Parameters to be set in the runSimulation function are whether or not the user wants to see the animations,
the perc_fill sets how crowded the goal locations might be, and the parameters regarding the nb of agents, maps
spawn types can also be set. Additionally, by setting plotVar to true the user can see the graph for coefficient of variation. 

To use a different seed, the user can change line 4 in create_sim2.py

The statistical tests are generated in statistics.py, which is not run by the mentioned command.

The results of the simulation are stored in save_dictionary_SOLVERNAME. Note however, that the code might take a while to run. In findSolution methods the maximum acceptable run time can be set, which is recommended as especially CBS tends to take a really long time to run. 

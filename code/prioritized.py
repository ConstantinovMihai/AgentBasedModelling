import time as timer
from single_agent_planner import computeHeuristics, a_star, getSumOfCost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(computeHeuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []      
        
        for i in range(self.num_of_agents):  # Find path for each agent
            
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            # if no solution is found for this agent, return an empty list and do not attempt to solve for future agents
            # the empty list is used to detect that A* could not find solutions
            if path is None:
                result = []
                print('path none')
                self.CPU_time = 0
                return result, self.CPU_time
                break
            else:            
            # path needs to be trimmed since constraints added for 200 future time steps will make paths unnecessarily long which will influence total cost
            # paths where agent stays at same location for the remainder of time are trimmed
                trim_length = 0
                found = False
                while found == False:
                    if path[-trim_length-1] != path[-trim_length-2]:
                        found = True
                    else:
                        trim_length += 1

                path2 = path[:len(path)-trim_length]
                result.append(path2)
                # if the length of the path is very large, as agent has to wait very long, it is assumed no solution can be found and an empty list is returned
                if len(path)> 500:
                    result = []
                    self.CPU_time = 0
                    return result, self.CPU_time
                    break

            # for each agent after current agent        
            for j in range(i+1,self.num_of_agents):
                # for paths of all previous agents
                for path in result:
                    # for each location of previous agents
                    for t in range(0,len(path)):
                        # if the last location of that agent, implement constraint for next 100 time steps
                        if  t == len(path)-1:
                            for constraint_time in range(t,100):
                                constraints.append({'agent': j,'loc': [path[t]],'timestep': constraint_time})
                        # else implement for current time step
                        else:
                            constraints.append({'agent': j,'loc': [path[t]],'timestep': t})
                        # if not the agents last point, implement an edge constraint as well
                        if t < len(path)-1:
                            constraints.append({'agent': j,'loc': [path[t+1],path[t]],'timestep': t+1})

        self.CPU_time = timer.time() - start_time
        
        # do not penalize the cpu time fo the method for sitatuions where no results was found anyways
        if not result:
            self.CPU_time = 0

        # print("\n Found a solution! \n")
        # print("CPU time (s):    {:.2f}".format(self.CPU_time))
        # print("Sum of costs:    {}".format(getSumOfCost(result)))
      
        return result, self.CPU_time

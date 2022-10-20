import heapq


def move(loc, dir):
    # Included (0,0) direction to allow agents to wait in certain cell
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0,0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def getSumOfCost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def computeHeuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def buildConstraintTable(constraints, agent):
    """
    Return a table that constains the list of constraints of
                   the given agent for each time step. The table can be used
                   for a more efficient constraint violation check in the 
                   is_constrained function.
    """
    constraint_table = []
    max_time = 0

    # for each constraint in the list of constraints:
    for constraint in constraints:  
        # if the constraint in the list is intended for the current agent:      
        if constraint["agent"] == agent:
            # add the constraint location and timestep into the temporary constraint table for this agent
            constraint_table.append([constraint["loc"],constraint["timestep"]])
            # if the constraint time is larger than the current indentified max timestep for any constraint, update the max time 
            if constraint["timestep"] >= max_time:
               max_time = constraint["timestep"]
    
 
    indexed_constraint_table = []
    # create indexed constraint table with length of largest constraint timestep
    for i in range(max_time+1):
        indexed_constraint_table.append([])
    
    # for each constraint identified for this agent:
    for constraint in constraint_table:
        # insert constraint's location at the constraints timestep
        indexed_constraint_table[constraint[1]].append(constraint[0])    
   
    return indexed_constraint_table


def getLocation(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def getPath(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def isConstrained(curr_loc, next_loc, next_time, constraint_table):
    constrained = False
    
    # check for times earlier than the latest constraint
    if next_time < len(constraint_table):   
        # check each constraint for agent at next time
        for constraint in constraint_table[next_time]:
            # if constraint saved as a vertex constraint
            if len(constraint) == 1:
                # if constraint location equals next location
                if constraint[0] == next_loc:
                    constrained = True
            # if constraint is saved as a edge constraint
            if len(constraint) == 2:
                # if current and next location match constraint locations
                if constraint[0] == curr_loc and constraint[1] == next_loc:
                    constrained = True
                              
    return constrained


def pushNode(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def popNode(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compareNodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, time = 0, distributed = False):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the ID of the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    
    # build constraint table for this agent
    indexed_constraint_table = buildConstraintTable(constraints, agent)
    
    

    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 't_step':time}
    pushNode(open_list, root)
    closed_list[(root['loc'],root['t_step'])] = root
    while len(open_list) > 0:
        curr = popNode(open_list)
    
        # if agent has reached goal location and there are no constraints posed on agent at a later time:
        # in the case of the distributed solver, once the goal is reached, the path is returned, further constraints are ignored     
        if curr['loc'] == goal_loc and len(getPath(curr)) >= len(indexed_constraint_table) and distributed == False:
            return getPath(curr)

        elif curr['loc'] == goal_loc and distributed == True and not isConstrained(curr['loc'], curr['loc'], curr['t_step']+1, indexed_constraint_table):
        
            return getPath(curr)

        for dir in range(5):
            child_loc = move(curr['loc'], dir)            
            # if child location is outside of map
            if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]):
                continue
            # if child location is a blocked cell or the cell is constrained
            if my_map[child_loc[0]][child_loc[1]] or isConstrained(curr['loc'], child_loc, curr['t_step']+1, indexed_constraint_table): 
                continue            
    
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    't_step': curr['t_step'] + 1}           
             
            
            if (child['loc'],child['t_step']) in closed_list :
                existing_node = closed_list[(child['loc'],child['t_step'])]
                if compareNodes(child, existing_node):
                    closed_list[(child['loc'],child['t_step'])] = child
                    pushNode(open_list, child)
            else:
                closed_list[(child['loc'],child['t_step'])] = child
                pushNode(open_list, child)

   
    return None  # Failed to find solutions

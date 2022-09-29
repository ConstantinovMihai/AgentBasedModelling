import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, push_node


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    
    # for each timestep in longest path
    for ts in range(max(len(path2),len(path1))):
        # identify vertex collision
        if get_location(path1, ts) == get_location(path2, ts):
            return [get_location(path1, ts)], ts
        # identify edge collisions if one of the agents is still moving
        elif get_location(path1, ts-1) == get_location(path2, ts) and get_location(path2, ts-1) == get_location(path1, ts) and ts -1 < min(len(path2),len(path1)):
            return [get_location(path1, ts),get_location(path2, ts)], ts
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    # for each path
    for i in range(len(paths)):
        # compare path with subsequent paths
        for j in range(i+1,len(paths)):
            # if collsion is detected
            if detect_collision(paths[i], paths[j]) != None:
                location, t = detect_collision(paths[i], paths[j])
                # append collision
                collisions.append({'a1': i, 'a2': j, 'loc': location, 'timestep': t})
            
            
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    #{'a1': i, 'a2': j, 'loc': location, 'timestep': t}
    constraints = []
    
    # if collsion is a vertex collision
    if len(collision['loc']) == 1:        
        # create constraint for agent 0
        constraints.append({'agent': collision['a1'],'loc': collision['loc'],'timestep': collision['timestep']})
        # create constraint for agent 1
        constraints.append({'agent': collision['a2'],'loc': collision['loc'],'timestep': collision['timestep']})
    # if collision is an edge collision
    elif len(collision['loc']) == 2:
        # create constraint for agent 0
        constraints.append({'agent': collision['a2'],'loc': collision['loc'],'timestep': collision['timestep']})
        # create constraint for agent 1 and flip order of locations
        constraints.append({'agent': collision['a1'],'loc': [collision['loc'][1],collision['loc'][0]],'timestep': collision['timestep']})
       
    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        #print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        
        self.push_node(root)

        '''# Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))'''
        
        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        
        while len(self.open_list) > 0:        
            # get next node with smallest cost
            P = self.pop_node()  
            # if node has no collisions, return paths         
            if len(P['collisions']) == 0:
                self.CPU_time = timer.time() - self.start_time
                return P['paths'], self.CPU_time

            # convert collision to list of two constraints         
            collision = P['collisions'][0]  
            constraints = standard_splitting(collision)          
            
            # for each constraint option, create new child
            for constraint in constraints:
                # child inherets neccessary properties
                new_list = P['constraints'].copy()
                new_list.append(constraint)
            
                Q = {'cost': 0,
                'constraints': new_list,
                'paths': P['paths'].copy(),
                'collisions': []}
                ai = constraint['agent']
                
                # create path for child including new constraint           
                path = a_star(self.my_map, self.starts[ai], self.goals[ai], self.heuristics[ai],
                          ai, Q['constraints'])                
                
                if path != None:
                                        
                    Q['paths'][ai] = path                                      
                    Q['collisions'] = detect_collisions(Q['paths'])                                       
                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    
                    
                    self.push_node(Q)
                    
            #i +=1

        #self.print_results(root)
        #return root['paths']
        


    def print_results(self, node):
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

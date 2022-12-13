import heapq

def move(loc, dir):
    # add (0,0) at idx 4 to make agent remain in place
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
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


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    table = dict()
    for constraint in constraints:
        # check for positive time steps
        if not ("positive" in constraint.keys()):
            constraint["positive"] = False            
        if constraint["agent"] == agent and not constraint["positive"]:
            if constraint["timestep"] not in table:
                table[constraint["timestep"]] = []
            table[constraint["timestep"]].append(constraint)
    return table

def build_positive_constraint_table(constraints, agent):
    ##############################
    # Task 4.1: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    table = dict()
    for constraint in constraints:
        # check for positive time steps
        if not ("positive" in constraint.keys()):
            constraint["positive"] = False            
        if constraint["agent"] == agent and constraint["positive"]:
            if constraint["timestep"] not in table:
                table[constraint["timestep"]] = []
            table[constraint["timestep"]].append(constraint)
    return table

def get_safeInterval(curr_loc, config, timestep, constraint_table):
    prev_time = None
    safeInterval = []
    set_Ofconstraint = []
    repeatingConstraint = []
    
    for(time, constraints) in constraint_table.items():
        for constraint in constraints:
            #checking for edge collision
            if len(constraint['loc']) != 1:
                if curr_loc == constraint['timestep'] not in Set_Ofconstraint and config == constraint['loc'][1] and constraint['loc'][0]:
                    set_Ofconstraint.append(constraint['timestep']-1)
                    set_Ofconstraint.append(constraint['timestep'])
            else:
            #checking for vertex collision
                if constraint['loc'][0] == config:
                    if time!= -1:
                        if constraint['timestep'] not in set_Ofconstraint:
                            set_Ofconstraint.append(constraint['timestep'])
                         
        set_Ofconstraint.sort()
        set_Ofconstraint += repeatingConstraint
        
        # when there are no constraints
        if len(set_Ofconstraint) == 0
            return [(timestep, -1)]
        
        for i in range(len(set_Ofconstraint)):
            if i==0 and type(set_Ofcontraint[i-1] == type(1) and set_Ofconstraint[i] > timestep:
                safeInterval.append((timestep, constraint_set[i]-1))
                             
            if i!=0 and type(constraint_set[i-1]) == type(1) and set_Ofconstraint[i-1]+1 != set_Ofconstraint[i]:
                if set_Ofconstraint[i-1]+1 <= timestep <= set_Ofconstraint[i]-1:
                             safeInterval.append((timestep, set_Ofconstraint[i]-1))
                elif timestep < set_Ofconstraint[i]
                             safeInterval.append((set_Ofconstraint[i-1]+1, set_Ofconstraint[i]-1))
            
            if i == len(setOfconstraint)-1 and set_Ofconstraint[i] != prev_time:
                if timestep > set_Ofconstraint[i]+1:
                             safeInterval.append((timestep, -1))
                else:
                             safeInterval.append((set_Ofconstraint[i]+1, -1))
            
    safeInterval.sort(key=lambda x:x[0])
    return safeInterval
   
def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time in constraint_table:
        for constraint in constraint_table[next_time]:
            if [next_loc] == constraint['loc']:    
                return True
            elif [curr_loc, next_loc] == constraint['loc']:
                return True
    return False

def is_constrained_positive(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 4.1: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if next_time in constraint_table:
        for constraint in constraint_table[next_time]:
            if [next_loc] == constraint['loc']:    
                return True
            elif [curr_loc, next_loc] == constraint['loc']:
                return True
    else:
        return True
    # if constraint[loc] doesnt match
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star_with_safe_intervals(my_map, start_loc, goal_loc, h_values, agent, constraints):

    open_list = []
    closed_list = dict()
  
    h_value = h_values[start_loc]
   
    #if the goal location is in no way connected to the start node then we must return a no solution
    if start_loc not in h_values_keys():
        return None
        
    constraint_table = build_constraint_table(constraints, agent)
   
    root_safe_interval = get_safeInterval() ####yet to make the safe interval fxn
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0, 'safe_interval': root_safe_interval, 'wait_time': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root_safe_interval)] = root
    
    maximum_time = 0
    if len(constraint_table.keys()) > 0:
        maximum_time = max(constraint_table.keys())
        
    #####write something to avoid infinite loop
    curr = pop_node(open_list)
    if (curr['loc'] == goal_loc and curr['timestep'] >= maximum_time):
           return get_path(curr)
           
     successors = get_successors(curr) ###yet to make the get successor fxn
    
    for successor in successors:
        #expand the closed list and root in a node with f-value less than the current one
    
    
    
    return None  # Failed to find solutions

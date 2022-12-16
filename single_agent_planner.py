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

def get_safeInterval(curr_loc, config, timestep, constraint_table):
    """
    This function returns the safe interval for a given vertex
    ----------
    curr_loc: current location of the agent
    config: the vertex that is being checked
    timestep: the timestep of the vertex
    constraint_table: the constraint table for the agent
    """

    


    prev_time = None
    safeInterval = []
    set_Ofconstraint = []
    repeatingConstraint = []
    
    for(time, constraints) in constraint_table.items():
        for constraint in constraints:
            #checking for edge collision
            if len(constraint['loc']) != 1:
                if curr_loc == constraint['timestep'] not in set_Ofconstraint and config == constraint['loc'][1] and constraint['loc'][0]:
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
        if len(set_Ofconstraint) == 0:
            return [(timestep, -1)]
        
        for i in range(len(set_Ofconstraint)):
            if i==0 and type(set_Ofconstraint[i-1]) == type(1) and set_Ofconstraint[i] > timestep:
                safeInterval.append((timestep, set_Ofconstraint[i]-1))
                             
            if i!=0 and type(set_Ofconstraint[i-1]) == type(1) and set_Ofconstraint[i-1]+1 != set_Ofconstraint[i]:
                if set_Ofconstraint[i-1]+1 <= timestep <= set_Ofconstraint[i]-1:
                    safeInterval.append((timestep, set_Ofconstraint[i]-1))
                elif timestep < set_Ofconstraint[i]:
                    safeInterval.append((set_Ofconstraint[i-1]+1, set_Ofconstraint[i]-1))
            
            if i == len(set_Ofconstraint)-1 and set_Ofconstraint[i] != prev_time:
                if timestep > set_Ofconstraint[i]+1:
                    safeInterval.append((timestep, -1))
                else:
                    safeInterval.append((set_Ofconstraint[i]+1, -1))
            
    safeInterval.sort(key=lambda x:x[0])
    return safeInterval

def earliest_arrival(safe_int, curr_time):
    # curr_time is greater than the time of safe_int
    if safe_int[1] != -1 and curr_time > safe_int[1]:
        return None
    # wait
    wait_time = safe_int[0] - curr_time
    if wait_time > 1:
        return wait_time 
    else:
        return 1

def get_successors(curr_node , my_map, h_value , constraint_table):
    successors = []
    
    for dir in range(4):
        x = move(curr_node['loc'], dir)
        # set boudary constraint 
        if x[0] < 0 or x[1] < 0 or x[0] >= len(my_map) or x[1] >= len(my_map[0]): 
            continue
        # encounter a block
        if my_map[x[0]][x[1]]:      
            continue
            
        m_time = 1
        start_t = curr_node['timestep'] + m_time
        end_t = curr_node['safe_interval'][1]
        if curr_node['safe_interval'][1] != -1:
            end_t = curr_node['safe_interval'][1] + 1
        x_safe_intervals = get_safeInterval(curr_node['loc'], x, curr_node['timestep']+1, constraint_table)
        for i in x_safe_intervals:
            if (i[0] > end_t and end_t != -1) or (i[1] < start_t and i[1] != -1):
                continue

            t = earliest_arrival(i, curr_node['timestep'])
            if t is None or t > end_t:
                continue

            successor = {'loc': x, 'g_val': curr_node['g_val']+t, 'h_val': h_value[x], 'parent': curr_node, 'timestep': curr_node['timestep']+t, 'safe_interval': i, 'wait_time': 0}
            if t > 1:
                successor['wait_time'] += t - 1 
            successors.append(successor)

    return successors


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]

    goal_locations = [c for c in constraints if type(c)==tuple]
    # filter constraints for goal locations
    constraints = [x for x in constraints if x not in goal_locations]

    constraint_table = build_constraint_table(constraints, agent)
    positive_constraint_table = build_positive_constraint_table(constraints, agent)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}

    # calc time limiting constraints
    if len(constraint_table.keys()) > 0:
        earliest_goal_time_step = max(constraint_table.keys())
    else:
        earliest_goal_time_step = 0

    # add timelimitting (Task 2.4)
    time_upper_bound = len(my_map)*len(my_map[0]) + earliest_goal_time_step

    push_node(open_list, root)
    closed_list[(root['loc'],0)] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)

        if curr['timestep'] > time_upper_bound:
            break
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and curr['timestep'] > earliest_goal_time_step:  
            return get_path(curr)
            #Add child node 4 for when agent waits
        for dir in range(5): 
            child_loc = move(curr['loc'], dir)

            # check if child loc exceeds map boundaries
            if child_loc[0] < 0 or  child_loc[1] < 0 or  child_loc[0] >= len(my_map) or  child_loc[1] >= len(my_map[0]):
                continue

            # check if child loc is True (blocked)    
            if my_map[child_loc[0]][child_loc[1]]:
                continue

            # check if child loc violates goal constraints
            if child_loc in goal_locations:
                continue

            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
                    
            #handle child violation of negative constraints
            if is_constrained(curr['loc'], child['loc'], child['timestep'], constraint_table):
                continue

            #handle child violation of positive constraints
            if not is_constrained_positive(curr['loc'], child['loc'], child['timestep'], positive_constraint_table):
                continue

            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'],child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'],child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'],child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions

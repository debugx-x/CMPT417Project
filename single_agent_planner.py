import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
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
    root = {"loc": goal, "cost": 0}
    heapq.heappush(open_list, (root["cost"], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if (
                child_loc[0] < 0
                or child_loc[0] >= len(my_map)
                or child_loc[1] < 0
                or child_loc[1] >= len(my_map[0])
            ):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {"loc": child_loc, "cost": child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node["cost"] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node["cost"]
    return h_values


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    """
    This function reconstructs the path from the goal node
    ----------
    goal_node: the goal node
    """
    # Reconstruct the path from the goal node
    path = [goal_node["loc"]]
    curr = goal_node
    # check if the parent node is None
    if curr["parent"] == None:
        return path

    # add the location of the parent node to the path
    while curr["parent"] is not None:
        # for the number of wait time
        for i in range(curr["wait_time"] + 1):
            path.append(curr["parent"]["loc"])
        curr = curr["parent"]
    path.reverse()
    return path


def build_constraint_table(constraints, agent):
    """
    This function builds a constraint table for the agent
    ----------
    constraints: the constraints for the agent
    agent: the agent
    """
    table = dict()
    # iterate through the constraints
    for constraint in constraints:
        # check if the constraint is for the agent
        if constraint["agent"] == agent:
            # set time step to -1 if not given
            time_step = -1

            # check if constraint timestep is a integer
            if isinstance(constraint["timestep"], int):
                time_step = constraint["timestep"]

            # check if time step is in the table
            if time_step not in table.keys():
                table[time_step] = []

            # add constraint to the table
            table[time_step].append(constraint)
    return table


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    """
    This function checks if the next location is constrained
    ----------
    curr_loc: current location of the agent
    next_loc: next location of the agent
    next_time: next time step of the agent
    constraint_table: the constraint table for the agent
    """
    # check if the next time step is in the constraint table
    if next_time in constraint_table:
        # check if the next location is in the constraint table
        for constraint in constraint_table[next_time]:
            # check if the next location is in the constraint
            if [next_loc] == constraint["loc"]:
                return True
            # check if the current location and next location are in the constraint
            elif [curr_loc, next_loc] == constraint["loc"]:
                return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node['safe_interval'], node))


def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1["g_val"] + n1["h_val"] < n2["g_val"] + n2["h_val"]


def getSafeInterval(curr_loc, config, timestep, constraint_table):
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
    setOfConstraint = []
    repeatingConstraint = []

    for (time, constraints) in constraint_table.items():
        for constraint in constraints:
            # checking for edge collision
            if len(constraint["loc"]) != 1:
                # checking for edge collision
                if(curr_loc == constraint["timestep"] not in setOfConstraint and config == constraint["loc"][1] and constraint["loc"][0]):
                    setOfConstraint.append(constraint["timestep"] - 1)
                    setOfConstraint.append(constraint["timestep"])
            else:
                # checking for vertex collision
                if constraint["loc"][0] == config:
                    if time != -1:
                        if constraint["timestep"] not in setOfConstraint:
                            setOfConstraint.append(constraint["timestep"])
                    else:
                        if constraint["timestep"] not in repeatingConstraint:
                            repeatingConstraint.append(constraint["timestep"])

        # sorting the set of constraints
        setOfConstraint.sort()
        repeatingConstraint.sort()
        setOfConstraint += repeatingConstraint

        # checking for repeating constraints
        for i in range(len(setOfConstraint)):
            if i != 0 and setOfConstraint[i] == setOfConstraint[i - 1]:
                setOfConstraint.remove(setOfConstraint[i])

        # when there are no constraints
        if len(setOfConstraint) == 0:
            return [(timestep, -1)]

        # checking for safe interval
        for i in range(len(setOfConstraint)):
            # first meet point not no start loc, and time not past the first constraint
            if i == 0 and isinstance(setOfConstraint[i], int):
                if timestep < setOfConstraint[i]:
                    safeInterval.append((timestep, setOfConstraint[i] - 1))
                elif timestep > setOfConstraint[i] + 1:
                    safeInterval.append((timestep, -1))
                else:
                    safeInterval.append((setOfConstraint[i] + 1, -1))

            # not the first meet point and obstacle not stay in meet point for more than 1 timestep
            if (i != 0 and isinstance(setOfConstraint[i - 1], int) and setOfConstraint[i - 1] + 1 != setOfConstraint[i]):
                if setOfConstraint[i - 1] + 1 <= timestep <= setOfConstraint[i] - 1:
                    safeInterval.append((timestep, setOfConstraint[i] - 1))
                elif timestep < setOfConstraint[i]:
                    safeInterval.append((setOfConstraint[i - 1] + 1, setOfConstraint[i] - 1))
                elif timestep > setOfConstraint[i] + 1:
                    safeInterval.append((timestep, -1))
                else:
                    safeInterval.append((setOfConstraint[i] + 1, -1))

            # not the first meet point and obstacle stay in meet point for more than 1 timestep and not the last meet point
            # and time not past the next meet point        
            if i == len(setOfConstraint) - 1 and setOfConstraint[i] != prev_time:
                if timestep > setOfConstraint[i] + 1:
                    safeInterval.append((timestep, -1))
                else:
                    safeInterval.append((setOfConstraint[i] + 1, -1))

            prev_time = setOfConstraint[i]

    safeInterval.sort(key=lambda x: x[0])
    return safeInterval


def earliest_arrival(safe_int, curr_time):
    """
    This function returns the earliest arrival time of a given vertex
    ----------
    safe_int: the safe interval of the vertex
    curr_time: the timestep of the vertex
    """
    # curr_time is greater than the time of safe_int or safe_int is not valid
    if safe_int[1] != -1 and curr_time > safe_int[1]:
        return None
    # wait until the safe interval
    if safe_int[0] > curr_time:
        return safe_int[0] - curr_time
    # wait for 1 timestep
    return 1


def get_successors(curr_node, my_map, h_value, constraint_table):
    """
    This function returns the successors of a given vertex
    ----------
    curr_node: the vertex that is being checked
    my_map: the map of the environment
    h_value: the heuristic value of the vertex
    constraint_table: the constraint table for the agent
    """
    successors = []
    # check for all 4 directions
    for dir in range(4):
        x = move(curr_node["loc"], dir)

        # check if next loc is out of bound
        if x[0] < 0 or x[1] < 0 or x[0] >= len(my_map) or x[1] >= len(my_map[0]):
            continue

        # encounter a block 
        if my_map[x[0]][x[1]]:
            continue

        # check for collision
        m_time = 1 if dir == curr_node["dir"] else 2 # moving time for the agent to move to the next loc
        
        start_t = curr_node["timestep"] + m_time
        end_t = curr_node["safe_interval"][1]

        # check if the safe interval of the current loc is valid
        if curr_node["safe_interval"][1] != -1:
            end_t = curr_node["safe_interval"][1] + 1

        # get the safe interval of the next loc
        safe_intervals = getSafeInterval(curr_node["loc"], x, curr_node["timestep"] + 1, constraint_table)

        for i in safe_intervals:

            # check if the safe interval is valid
            if (i[0] > end_t and end_t != -1) or (i[1] < start_t and i[1] != -1):
                continue

            # get the earliest arrival time
            t = earliest_arrival(i, curr_node["timestep"])

            # check if the earliest arrival time is valid
            if t is None or t > end_t:
                continue

            # create the successor
            successor = {
                "loc": x,
                "g_val": curr_node["g_val"] + t,
                "h_val": h_value[x],
                "parent": curr_node,
                "timestep": curr_node["timestep"] + t,
                "safe_interval": i,
                "wait_time": 0,
            }

            # check if the agent has to wait
            if t > 1:
                successor["wait_time"] += t - 1
            
            # add the successor to the list
            successors.append(successor)
    return successors


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """
    my_map      - binary obstacle map
    start_loc   - start position
    goal_loc    - goal position
    agent       - the agent that is being re-planned
    constraints - constraints defining where robot should or cannot go at each timestep
    """

    open_list = []
    closed_list = dict()

    #if the goal location is in no way connected to the start node then we must return a no solution
    if start_loc not in h_values.keys():
        return None

    # get the heuristic value of the start loc
    h_value = h_values[start_loc]

    # build the constraint table
    constraint_table = build_constraint_table(constraints, agent)

    # get root safe interval
    rootSafeInterval = getSafeInterval(start_loc, start_loc, 0, constraint_table)[0]

    # build the root node
    root = {
        "loc": start_loc,
        "g_val": 0,
        "h_val": h_value,
        "parent": None,
        "timestep": 0,
        "safe_interval": rootSafeInterval,
        "wait_time": 0,
    }

    # calc time limiting constraints
    if len(constraint_table.keys()) > 0:
        earliest_goal_time_step = max(constraint_table.keys())
    else:
        earliest_goal_time_step = 0

    # add timelimitting (Task 2.4)
    time_upper_bound = len(my_map) * len(my_map[0]) + earliest_goal_time_step

    # add root to open list
    push_node(open_list, root)

    # add root to closed list
    closed_list[(root["loc"], rootSafeInterval)] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)

        if curr["timestep"] > time_upper_bound:
            break

        if curr["loc"] == goal_loc and curr["timestep"] > earliest_goal_time_step:
            return get_path(curr)

        # expand the current node and get the successors
        successors = get_successors(curr, my_map, h_values, constraint_table)

        for succ in successors:
            # check if the successor is in the closed list
            if (succ["loc"], succ["safe_interval"]) in closed_list.keys():
                # check if the successor is better than the existing node
                if closed_list[(succ["loc"], succ["safe_interval"])]["g_val"] <= succ["g_val"]:
                    continue
                # check if the successor is in the open list
                if (succ['loc'], succ['safe_interval']) in closed_list:
                    existing_node = closed_list[(succ['loc'], succ['safe_interval'])]
                    # check if the successor is better than the existing node
                    if compare_nodes(succ, existing_node):
                        closed_list[(succ['loc'], succ['safe_interval'])] = succ
                        push_node(open_list, succ)
                else:
                    closed_list[(succ['loc'], succ['safe_interval'])] = succ
                    push_node(open_list, succ)

        time_upper_bound -= 1

    # No solutions
    return None  
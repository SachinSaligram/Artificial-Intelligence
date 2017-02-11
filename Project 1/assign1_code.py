# -----------------------------------------------------------------------------------------------------------
# Description: This is the code for question 3 of assignment 1 (CSC 520). Run this code to obtain: 
# 			   1. the nodes expanded and the path from urziceni to mehadia.
# 			   2. pair of cities for which BFS outperforms DFS and DFS outperforms A* 
# 			      in terms of the number of nodes expanded.
# Please change the filepath before running the code.
# -----------------------------------------------------------------------------------------------------------


# Import python standard libraries
import math, heapq
# Change/check the filepath before running the code
filepath = ''

# Open the file and read the lines. Perform operations to obtain data in the required format  
f = open(filepath+'roads')
lines = f.readlines()
f.close()
data = [tuple((line.rstrip('\n')).split(',')) for line in lines]

# Store the distances between any 2 cities
edge_weights = data[0:23]
# Store the latitude and longitude coordinates for each city
lat_long = data[23:43]

# Initialize variables required during execution
# Python dictionary representing a adjacency list for cities 
source_dest = {}
# Road distance between any 2 cities
cost = {}
# Straight line distance between any two cities
direct_dist = {}
# Unique list of cities
cities = []

# Create a Python dictionary representing a adjacency list for cities, and cost (road distance) between 2 cities
# Both are creating assuming the road is two-way. 
for source, destination, dist in edge_weights:
	if source not in source_dest.keys():
		source_dest[source] = []
		source_dest[source].append(destination)
	else:
		source_dest[source].append(destination)
	if destination not in source_dest.keys():
		source_dest[destination] = []
		source_dest[destination].append(source)
	else:
		source_dest[destination].append(source)
	cost[source,destination] = dist
	cost[destination,source] = dist

# Create a (key, tuple) dictionary to look up latitude and longitude coordinates for each city
for loc, lat, lon in lat_long:
	direct_dist[loc] = (lat,lon)

# Create a unique list of cities
for x in lat_long:
	cities.append(x[0])

# Depth First Search (DFS) function to discover all nodes expanded, number of nodes expanded, and the path returned from start to goal
# Return all three results as a list
def dfs(graph, start, goal, path, nodes):
    value = 0
    path += [start]
    if start == goal:
        return list(set(nodes)), path, (len(list(set(nodes)))+1)
    else:
        for node in graph[start]:
            if node not in path:										
                nodes.append(node)
                value = dfs(graph,node,goal,path[:],nodes[:])
                if value:
                    return value

# Breadth First Search (BFS) function to discover all nodes expanded, number of nodes expanded, and the path returned from start to goal
# Return all three results as a list
def bfs(graph, start, goal):
    queue = [(start, [start])]
    nodes = []
    while queue:
        (curr_node, path) = queue.pop(0)
        nodes.append(curr_node)
        for node in list(set(graph[curr_node]) - set(path)):
            if node == goal:
                return list(set(nodes + [node])), (path + [node]), len(list(set(nodes + [node])))
            else:
                queue.append((node, path + [node]))

# Heuristic function to compute the straight line distance, using latitude and longitude coordinates, between a city and the goal
def heuristic(loc1, loc2):
    (x1, y1) = direct_dist[loc1]
    (x2, y2) = direct_dist[loc2]
    euclid_dist = round(math.sqrt(((float(x2) - float(x1))**2) + ((float(y2) - float(y1))**2)),5)
    dist = int(round(euclid_dist/(0.00001*1000),0))
    return dist

# A* algorithm to discover all nodes expanded, number of nodes expanded, and the path returned from start to goal
# Return all three results as a list
def a_star(graph, start, goal):
    node_list = []
    nodes = []
    heapq.heappush(node_list, (0,start))
    path = []
    path_cost = {}
    path_cost[start] = 0
    path = {}
    path[start] = [start]

    while node_list:
        curr_node_pop = heapq.heappop(node_list)
        curr_node = curr_node_pop[1]
        nodes.append(curr_node)
        if curr_node == goal:
            return list(set(nodes)), path[curr_node], len(list(set(nodes)))
            break 
        for node in graph[curr_node]:
            node_cost = path_cost[curr_node] + int(cost[curr_node, node])
            if node not in path_cost or node_cost < path_cost[node]:
                path_cost[node] = node_cost
                priority = node_cost + heuristic(goal, node)
                #nodes.append(node)
                path[node] = path[curr_node]+[node]
                heapq.heappush(node_list,(priority,node))



# Answer to question 3a)
print
print "Answer to question 3a)"

print
start = 'urziceni'
goal = 'mehadia'
path = []
nodes = []

print "Nodes expanded and path returned by BFS"
print
BFS = bfs(source_dest, start, goal)
print "Nodes expanded:", BFS[0]
print
print "Path returned:", BFS[1]
print
print "================================================================="
print
print "Nodes expanded and path returned by DFS"
DFS = dfs(source_dest, start, goal, path, nodes)
print "Nodes expanded:", DFS[0]+[start]
print
print "Path returned:", DFS[1]


bfs_over_dfs = 0
dfs_over_astar = 0
bfs_dfs_cities = []
dfs_astar_cities = []

# Iterate over cities
for start in cities:
    for goal in cities:
        path = []
        nodes = []
        if start == goal:                                           # If start is same as goal, skip the subsequent steps
            continue
        else:
            BFS = bfs(source_dest, start, goal)                     # Call bfs function to return the number of nodes expanded
            DFS = dfs(source_dest, start, goal, path, nodes)        # Call dfs function to return the number of nodes expanded
            astar = a_star(source_dest, start, goal)                # Call astar function to return the number of nodes expanded
            
            if (BFS is None) or (DFS is None):                      # If bfs of dfs return type 'None', skip the subsequent steps
                continue
            else:
                if BFS[2] < DFS[2]:                                 # Compare the number of nodes expanded 
                    bfs_over_dfs += 1                               # Count such instances
                    bfs_dfs_cities.append((start, goal, BFS[0], DFS[0], BFS[2], DFS[2]))            # Store the pair of cities meeting the required criterion
                    #break

            if (DFS is None) or (astar is None):                    # If dfs of astar return type 'None', skip the subsequent steps
                continue
            else:
                if DFS[2] < astar[2]:                               # Compare the number of nodes expanded
                    dfs_over_astar += 1                             # Count such instances
                    dfs_astar_cities.append((start, goal, DFS[0], astar[0], DFS[2], astar[2]))          # Store the pair of cities meeting the required criterion
                    #break


# Answer to question 3b) and 3c)
print
print "Answer to question 3b) and 3c)"

print
print "There are", bfs_over_dfs, "pairs of cities where BFS outperforms DFS in terms of number of nodes expanded. One of the pairs are shown below."
print "Pair of cities:", (bfs_dfs_cities[0][0],bfs_dfs_cities[0][1])
print "Nodes expanded (including destination) for BFS:", bfs_dfs_cities[0][2]
print "Nodes expanded (including destination) for DFS:", bfs_dfs_cities[0][3]
print "Number of nodes expanded for BFS:", bfs_dfs_cities[0][4]
print "Number of nodes expanded for DFS:", bfs_dfs_cities[0][5]
#for pair in bfs_dfs_cities:
    #print pair

print
print "There are", dfs_over_astar, "pairs of cities where DFS outperforms A* in terms of number of nodes expanded. One of the pairs are shown below."
print "Pair of cities:", (dfs_astar_cities[0][0],dfs_astar_cities[0][1])
print "Nodes expanded (including destination) for DFS:", dfs_astar_cities[0][2]
print "Nodes expanded (including destination) for A*:", dfs_astar_cities[0][3]
print "Number of nodes expanded for DFS:", dfs_astar_cities[0][4]
print "Number of nodes expanded for A*:", dfs_astar_cities[0][5]
#for pair in dfs_astar_cities:
    #print pair

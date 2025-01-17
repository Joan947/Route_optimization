import networkx as nx
import matplotlib.pyplot as plt
import geopandas as gpd
import pickle
import heapq
import math
import random
# Bidirectional A* search with risk factor 
# Still need to adpat code with the new heuristic 

# Constants for critical thresholds
CRITICAL_DEPTH = 0.3  # Critical depth threshold for safe passage in meters
CRITICAL_VELOCITY = 3  # Critical velocity threshold for safe passage in m/s



with open("graph.gpickle", "rb") as file:
    G = pickle.load(file)

def euclidean_distance(x1, y1, x2, y2):
    """
    Calculate Euclidean distance between two points.
    """
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def risk(depth, velocity):
    """
    Calculate the risk factor for a flooded area based on depth and velocity.
    """
    depth_risk = depth / CRITICAL_DEPTH
    velocity_risk = (velocity / CRITICAL_VELOCITY) ** 2
    return depth_risk + velocity_risk

def heuristic(node, goal, flood_penalty):
    """
    Heuristic with risk factor and Euclidean distance.
    """
    # Coordinates for current node and goal node
    x1, y1 = node
    x2, y2 = goal

    # Base heuristic using distance
    base_cost = euclidean_distance(x1, y1, x2, y2)

    # Risk factor and flood penalty
    depth = G.nodes[node].get("depth", 0)
    velocity = G.nodes[node].get("velocity", 0)
    risk_factor = risk(depth, velocity)
    flood_risk = flood_penalty.get(node, 0)

    return base_cost + risk_factor + flood_risk

def edge_cost(u, v):
    """
    Cost of traveling between nodes u and v.
    """
    length = G.edges[u, v].get("length", 1)
    return length

def bidirectional_a_star(G, start, goal, flood_penalty):
    # Priority queues for forward and reverse search
    forward_queue = [(0, start)]
    reverse_queue = [(0, goal)]

    # Costs and paths for forward and reverse search
    forward_costs = {start: 0}
    reverse_costs = {goal: 0}
    forward_parents = {start: None}
    reverse_parents = {goal: None}

    # Visited nodes
    forward_visited = set()
    reverse_visited = set()

    meeting_node = None
    best_cost = float("inf")

    while forward_queue and reverse_queue:
        # Forward search step
        f_cost, f_node = heapq.heappop(forward_queue)
        if f_node in reverse_visited:
            total_cost = forward_costs[f_node] + reverse_costs[f_node]
            if total_cost < best_cost:
                best_cost = total_cost
                meeting_node = f_node
            break  # Found meeting point
        
        forward_visited.add(f_node)
        for neighbor in G.neighbors(f_node):
            new_cost = forward_costs[f_node] + edge_cost(f_node, neighbor)
            if neighbor not in forward_costs or new_cost < forward_costs[neighbor]:
                forward_costs[neighbor] = new_cost
                forward_parents[neighbor] = f_node
                priority = new_cost + heuristic(neighbor, goal, flood_penalty)
                heapq.heappush(forward_queue, (priority, neighbor))
        
        # Reverse search step
        r_cost, r_node = heapq.heappop(reverse_queue)
        if r_node in forward_visited:
            total_cost = reverse_costs[r_node] + forward_costs[r_node]
            if total_cost < best_cost:
                best_cost = total_cost
                meeting_node = r_node
            break  # Found meeting point

        reverse_visited.add(r_node)
        for neighbor in G.neighbors(r_node):
            new_cost = reverse_costs[r_node] + edge_cost(r_node, neighbor)
            if neighbor not in reverse_costs or new_cost < reverse_costs[neighbor]:
                reverse_costs[neighbor] = new_cost
                reverse_parents[neighbor] = r_node
                priority = new_cost + heuristic(neighbor, start, flood_penalty)
                heapq.heappush(reverse_queue, (priority, neighbor))

    # Reconstruct path
    path = []
    if meeting_node:
        # Forward path from start to meeting point
        node = meeting_node
        while node:
            path.append(node)
            node = forward_parents[node]
        path.reverse()

        # Reverse path from meeting point to goal
        node = reverse_parents[meeting_node]
        while node:
            path.append(node)
            node = reverse_parents[node]
    
    return path, best_cost



with open("valid_nodes.pkl", "rb") as file:
    valid_nodes = pickle.load(file)

# print("Valid nodes:", valid_nodes)

# Randomly select start and goal nodes from the valid nodes
start_node = random.choice(valid_nodes)
goal_node = random.choice(valid_nodes)

print(f"Start Node: {start_node}")
print(f"Goal Node: {goal_node}")
flood_penalty = {
    (34.052235, -118.243683): 10,  # Replace with actual coordinates
    (40.712776, -74.005974): 15,  
}


path, cost = bidirectional_a_star(G,start_node, goal_node, flood_penalty)
print("Shortest Path is :", path)
print("Total Cost is :", cost)

# Need to set up visualization for the graph and code (do the plotting)
# Start testing code for written output then with gragh visualization 
# Need to do bidirectional without risk factor 

#Challenges:
#Integrating risk factor equation into heuristic 
#Using the gerographic coordinates in the code , changing paramters 
# Not sure how to visualize yet 




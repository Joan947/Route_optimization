
import matplotlib.pyplot as plt
#import geopandas as gpd
import heapq
import networkx as nx
import pickle
import random
import math
#from shapely.geometry import Point


with open("valid_nodes.pkl", "rb") as file:
    valid_nodes = pickle.load(file)

# print("Valid nodes:", valid_nodes)

# Randomly select start and goal nodes from the valid nodes
start_node = random.choice(valid_nodes)
goal_node = random.choice(valid_nodes)

print(f"Start Node: {start_node}")
print(f"Goal Node: {goal_node}")


# Define the heuristic function
def heuristic(node, goal, flood_penalty):
    base_cost = math.dist(node, goal)
    flood_risk = flood_penalty.get(node, 0)  # Default penalty = 0
    return base_cost + flood_risk


with open("graph.gpickle", "rb") as file:
    G = pickle.load(file)


# Define the edge cost function
def edge_cost(u, v):
    # Get the length attribute of the edge
    return G.edges[u, v]['length']

# Define the A* search algorithm
def a_star_search(start, goal, flood_penalty):
    if not isinstance(start, tuple) or not isinstance(goal, tuple):
        raise ValueError("Start and goal nodes must be tuples")
    # Priority queue for A* search
    queue = [(0, start)]
    g_costs = {start: 0}
    parents = {start: None}

    while queue:
        current_cost, current_node = heapq.heappop(queue)
        
        if current_node == goal:
            # Reconstruct path
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = parents[current_node]
            return path[::-1], g_costs[goal]

        for neighbor in G.neighbors(current_node):
            new_cost = g_costs[current_node] + edge_cost(current_node, neighbor)

            if neighbor not in g_costs or new_cost < g_costs[neighbor]:
                g_costs[neighbor] = new_cost
                parents[neighbor] = current_node
                priority = new_cost + heuristic(neighbor, goal, flood_penalty)
                heapq.heappush(queue, (priority, neighbor))

    return [], float("inf")

#This is for running tests for later

flood_penalty = {
    (34.052235, -118.243683): 10,  # Replace with actual coordinates
    (40.712776, -74.005974): 15,  
}


path, cost = a_star_search(start_node, goal_node, flood_penalty)
print("Shortest Path is :", path)
print("Total Cost is :", cost)



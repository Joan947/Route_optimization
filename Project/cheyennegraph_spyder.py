# -*- coding: utf-8 -*-
"""
Created on Fri Nov  8 10:09:37 2024

@author: Sean Castaneda
"""

import os
import geopandas as gpd
import networkx as nx
from shapely.geometry import Point, LineString
import matplotlib.pyplot as plt

#enter your directory here
directory = r"C:\Users\scastan1\OneDrive - University of Wyoming\IntroToAI\Project"
street_shape = "streets.shp"
streets = gpd.read_file(os.path.join(directory, street_shape))



# Create a graph from the shapefile
G = nx.Graph()

for idx, row in streets.iterrows():
    line = row['geometry']
    if line.geom_type == 'LineString':
        # Get the points along the LineString
        points = list(line.coords)
        
        # Loop through each segment in the LineString and add as an edge in the graph
        for i in range(len(points) - 1):
            start = points[i]
            end = points[i + 1]
            
            # Add edge with relevant attributes (you can add more if needed)
            G.add_edge(
                start, 
                end, 
                length=line.length, 
                speed_limit=row['speedlim'], 
                road_name=row['st_name']
            )

# Optional: Check the number of nodes and edges in your graph
print(f"Number of nodes: {G.number_of_nodes()}")
print(f"Number of edges: {G.number_of_edges()}")

pos = {node: node for node in G.nodes()}  # Use node coordinates for positioning
plt.figure(figsize=(10, 10))
nx.draw(G, pos, node_size=10, with_labels=False)
plt.show()

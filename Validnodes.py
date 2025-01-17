import pickle

# Load the graph from the pickle file
with open("graph.gpickle", "rb") as file:
    G = pickle.load(file)

# Filtering valid nodes (need to fix this later to make sure our filtering is correct based on our praramters)
valid_nodes = [node for node in G.nodes if isinstance(node, tuple) and len(node) == 2]

# Save valid nodes to a file 
with open("valid_nodes.pkl", "wb") as file:
    pickle.dump(valid_nodes, file)

print(f"Valid nodes saved to 'valid_nodes.pkl'")

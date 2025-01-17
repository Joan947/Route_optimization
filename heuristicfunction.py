
#risk factor (based on depth and velocity)

# Constants for critical depth and velocity thresholds
CRITICAL_DEPTH = 0.3  # Critical depth threshold for safe passage in meters
CRITICAL_VELOCITY = 3  # Critical velocity threshold for safe passage in m/s

def euclidean_distance(x1, y1, x2, y2):
    """
    Using Euclidean distance as a heuristic for this project
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

def road_heuristic(current_node, goal_node):
    """
    Heuristic function using only Euclidean distance.
    I assume we will use longitudes and latitudes as nodes for intersections
    """
    return euclidean_distance(current_node.lat, current_node.lon, goal_node.lat, goal_node.lon)
"""
    Assuming you have appended depth and velocity as attributes to nodes,
      and longitudes and latitudes are  attributes to nodes also, you can call the 
      road_heuristic() as h(n) and risk(n) and add them to g(n)
"""

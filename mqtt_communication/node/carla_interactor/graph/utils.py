import networkx as nx
import numpy as np

def find_nearest_node(graph, x, y, z):
    """
    Finds the nearest node in the graph to the given (x, y, z) coordinates.

    Assumes each node in the graph has 'x', 'y', and 'z' attributes and the graph is a NetworkX graph.
    """
    # Extracting the node attributes into a numpy array
    attributes = np.array([[data['x'], data['y'], data['z']] for _, data in graph.nodes(data=True)
                           if 'x' in data and 'y' in data and 'z' in data])
    
    if not attributes.size:
        return None  # Return None if no valid nodes are found

    # Compute differences from target coordinates
    differences = attributes - np.array([x, y, z])

    # Calculate the Euclidean distance from the target point
    distances = np.linalg.norm(differences, axis=1)

    # Get the index of the minimum distance
    nearest_node_index = np.argmin(distances)

    # Map back to node label
    valid_nodes = [node for node, data in graph.nodes(data=True) if 'x' in data and 'y' in data and 'z' in data]
    nearest_node = valid_nodes[nearest_node_index]

    return nearest_node


def find_shortest_path(graph, start_id, end_id):
    """
    Finds the shortest path between two waypoints in the graph using Dijkstra's algorithm.
    """
    try:
        path = nx.dijkstra_path(graph, start_id, end_id, weight='weight')
        return path
    except nx.NetworkXNoPath:
        print("No path found between the specified start and end waypoints.")
        return []


import json

class CARLAEncoder(json.JSONEncoder):
    def default(self, obj):
        if hasattr(obj, '__dict__'):
            return obj.__dict__
        if isinstance(obj, carla.Vector3D):
            return {'x': obj.x, 'y': obj.y, 'z': obj.z}
        return super().default(obj)

import json
import carla

class CARLAEncoder(json.JSONEncoder):
    def default(self, obj):
        if hasattr(obj, '__dict__'):
            return obj.__dict__
        if isinstance(obj, carla.Vector3D):
            return {'x': obj.x, 'y': obj.y, 'z': obj.z}
        return super().default(obj)

class MyClass:
    def __init__(self, header, roads, road_ids, junctions, junction_ids, landmarks):
        self.header = header
        self.roads = roads
        self.road_ids = road_ids
        self.junctions = junctions
        self.junction_ids = junction_ids
        self.landmarks = landmarks

    def to_dict(self):
        map_dict = {}
        map_dict['header'] = self.header.__dict__
        map_dict['roads'] = [road.__dict__ for road in self.roads]
        map_dict['road_ids'] = self.road_ids
        map_dict['junctions'] = [junction.__dict__ for junction in self.junctions]
        map_dict['junction_ids'] = self.junction_ids
        map_dict['landmarks'] = [landmark.__dict__ for landmark in self.landmarks]
        return map_dict

    def to_json(self, path):
        content = self.to_dict()
        with open(path, 'w') as f:
            json.dump(content, f, indent=4, cls=CARLAEncoder)

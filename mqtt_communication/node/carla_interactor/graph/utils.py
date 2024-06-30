import json
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt


def add_weights_to_edges(graph):
    # To every edge in the graph, add a weight attribute as the Euclidean distance between the two nodes
    for u, v in graph.edges():
        # Extract the vertex information for the two nodes
        u_vertex = np.array(graph.nodes[u]['vertex'])
        v_vertex = np.array(graph.nodes[v]['vertex'])
        # Calculate the Euclidean distance between the two nodes
        distance = np.linalg.norm(u_vertex - v_vertex)
        # Add the distance as the weight to the edge
        graph[u][v]['weight'] = distance

    return graph

def plot_graph(graph):
    """
    Plots the graph with nodes and edge weights.
    
    :param graph: networkx graph object
    """
    # Position nodes using a layout algorithm (like spring layout)
    pos = nx.spring_layout(graph)
    
    # Draw the nodes and edges with labels
    nx.draw_networkx_nodes(graph, pos, node_size=700)
    nx.draw_networkx_edges(graph, pos)
    
    # Edge labels: displaying the weights
    edge_labels = nx.get_edge_attributes(graph, 'weight')  # Assuming 'weight' is the attribute for edge weights
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels)
    
    # Node labels: displaying the node IDs with their coordinates (optional)
    node_labels = {node: str(node) for node in graph.nodes()}
    nx.draw_networkx_labels(graph, pos, labels=node_labels)
    
    # Set the title and show the plot
    plt.title('Graph Visualization with Nodes and Weights')
    plt.axis('off')  # Turn off the axis
    plt.show()


def localize(map, road_id_to_edge, x, y, z):
    location = carla.Location(x=x, y=y, z=z)
    waypoint = map.get_waypoint(location)
    edge = None
    try:
        edge = road_id_to_edge[waypoint.road_id][waypoint.section_id][waypoint.lane_id]
    except KeyError:
        pass
    return edge


def distance_heuristic(graph, n1, n2):
    l1 = np.array(graph.nodes[n1]['vertex'])
    l2 = np.array(graph.nodes[n2]['vertex'])
    return np.linalg.norm(l1-l2)

def path_search(graph, map, road_id_to_edge, x1, y1, z1, x2, y2, z2):
    """
    This function finds the shortest path connecting origin and destination
    using A* search with distance heuristic.
    origin      :   carla.Location object of start position
    destination :   carla.Location object of of end position
    return      :   path as list of node ids (as int) of the graph self._graph
    connecting origin and destination
    """
    start, end = localize(map, road_id_to_edge, x1, y1, z1), localize(map, road_id_to_edge, x2, y2, z2)
    heuristic = lambda n1, n2: distance_heuristic(graph, n1, n2)

    route = nx.astar_path(
        graph, source=start[0], target=end[0],
        heuristic=heuristic, weight='weight')
    route.append(end[1])
    return route

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

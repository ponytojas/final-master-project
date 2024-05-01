import matplotlib.pyplot as plt
import time
import os
import carla
import numpy as np
import networkx as nx

# Define logging level as logging.INFO
import logging
logging.basicConfig(level=logging.INFO)


def create_graph(carla_map):
    """
    Creates a directed NetworkX graph from a CARLA map's topology, ensuring all waypoints are included.
    """

    graph = nx.DiGraph()

    # First, add all waypoints as nodes to ensure they are included in the graph
    all_waypoints = carla_map.generate_waypoints(
        2.0)  # Adjust distance as per requirement
    for waypoint in all_waypoints:
        graph.add_node(waypoint.id)

    # Retrieve the topology - a list of (waypoint, next_waypoint) tuples
    topology = carla_map.get_topology()

    for waypoint, next_waypoint in topology:
        start_id = waypoint.id
        end_id = next_waypoint.id
        distance = waypoint.transform.location.distance(
            next_waypoint.transform.location)
        graph.add_edge(start_id, end_id, weight=distance)
        graph.add_node(waypoint.id, x=waypoint.transform.location.x,
                       y=waypoint.transform.location.y, z=waypoint.transform.location.z)
        if waypoint.id not in waypoints_locations:
            waypoints_locations[waypoint.id] = {'x': waypoint.transform.location.x,
                                                'y': waypoint.transform.location.y, 'z': waypoint.transform.location.z, 'transform': waypoint.transform}

    return graph


def create_graph_from_topology(topology):
    """
    Creates a directed NetworkX graph from a CARLA map's topology.
    """

    graph = nx.DiGraph()

    for waypoint, next_waypoint in topology:
        start_id = waypoint.id
        end_id = next_waypoint.id
        distance = waypoint.transform.location.distance(
            next_waypoint.transform.location)
        graph.add_edge(start_id, end_id, weight=distance)
        graph.add_node(waypoint.id, x=waypoint.transform.location.x,
                       y=waypoint.transform.location.y, z=waypoint.transform.location.z)
        graph.add_node(next_waypoint.id, x=next_waypoint.transform.location.x,
                          y=next_waypoint.transform.location.y, z=next_waypoint.transform.location.z)
        if waypoint.id not in waypoints_locations:
            waypoints_locations[waypoint.id] = {'x': waypoint.transform.location.x,
                                                'y': waypoint.transform.location.y, 'z': waypoint.transform.location.z, 'transform': waypoint.transform}
        

    return graph

def find_nearest_node(graph, x, y, z):
    """
    Finds the nearest node in the graph to the given (x, y, z) coordinates.

    Assumes each node in the graph has 'x', 'y', and 'z' attributes.
    """
    min_distance = np.inf
    nearest_node = None

    for node in graph.nodes(data=True):
        data = node[1]
        if 'x' not in data or 'y' not in data or 'z' not in data:
            continue
        node_x, node_y, node_z = data['x'], data['y'], data['z']
        distance = np.sqrt((node_x - x) ** 2 + (node_y - y)
                           ** 2 + (node_z - z) ** 2)

        if distance < min_distance:
            min_distance = distance
            nearest_node = node[0]

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


START_LOCATION = {'x': -110.7, 'y': -4.3, 'z': 0}
END_LOCATION = {'x': -21.6, 'y': 130, 'z': 0.5}
waypoints_locations = {}

# Connect to the default carla server
logging.info('Connecting to Carla server...')
client = carla.Client('localhost', 2000)
logging.info('Connected to Carla server')
client.set_timeout(2.0)

logging.info('Loading the map...')
world = client.get_world()
logging.info('Map loaded')

map = world.get_map()
# map_string = map.to_opendrive()
# map.save_to_disk('map.oxdr')
logging.info('Getting the topology...')
waypoint_tuple_list = map.get_topology()
logging.info('Topology obtained')

logging.info('Creating the graph from the topology...')
_graph = create_graph_from_topology(waypoint_tuple_list)
logging.info('Graph created')

# nx.draw(_graph)
# plt.show()


spawn_points = world.get_map().get_spawn_points()
initial_point = spawn_points[2]

start_time = time.time()

logging.info('INITIAL POINT TYPE: %s', type(initial_point))
start = find_nearest_node(
    _graph, initial_point.location.x, initial_point.location.y, initial_point.location.z)
end = find_nearest_node(
    _graph, END_LOCATION['x'], END_LOCATION['y'], END_LOCATION['z'])
route = find_shortest_path(_graph, start, end)




blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('vehicle.audi.a2')[0]
vehicle = world.spawn_actor(vehicle_bp, initial_point)


try:
    while True:
        world.tick()
except KeyboardInterrupt:
    vehicle.destroy()
    exit(0)

# # Spawn a vehicle
# blueprint_library = world.get_blueprint_library()
# vehicle_bp = blueprint_library.filter('vehicle.*')[0]
# spawn_points = world.get_map().get_spawn_points()

# transform = spawn_points[0]
# vehicle = world.spawn_actor(vehicle_bp, transform)

# start_time = time.time()

# graph = create_graph_from_topology(map)
# print("Graph created in", time.time() - start_time, "seconds")

# start = find_nearest_node(
#     graph, START_LOCATION['x'], START_LOCATION['y'], START_LOCATION['z'])
# end = find_nearest_node(
#     graph, END_LOCATION['x'], END_LOCATION['y'], END_LOCATION['z'])

# # Get a list of all graph nodes
# graph_nodes = list(graph.nodes)
# # Check if the start and end waypoints are in the graph
# if start not in graph_nodes:
#     print("Start waypoint not in the graph.")
# if end not in graph_nodes:
#     print("End waypoint not in the graph.")


# route = find_shortest_path(graph, start, end)

# print("Time taken:", time.time() - start_time)

# # Destroy the vehicle
# vehicle.destroy()

# # Plot the route
# route_x = []
# route_y = []
# for waypoint_id in route:
#     waypoint = waypoints_locations[waypoint_id]
#     route_x.append(waypoint['x'])
#     route_y.append(waypoint['y'])

# plt.figure()
# plt.plot(route_x, route_y, 'r')

# plt.show()

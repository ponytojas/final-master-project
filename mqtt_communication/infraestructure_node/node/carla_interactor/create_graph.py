import networkx as nx


def create_graph(carla_map):
    graph = nx.DiGraph()
    waypoints_locations = {}

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
            waypoints_locations[waypoint.id] = waypoint

    return graph, waypoints_locations

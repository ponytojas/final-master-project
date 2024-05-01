import networkx as nx


def create_graph(carla_map):
    graph = nx.DiGraph()
    waypoints_locations = {}

    topology = carla_map.get_topology()

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
            

    return graph, waypoints_locations
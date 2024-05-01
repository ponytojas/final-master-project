import os
import carla

from .create_graph import create_graph


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


class CarlaInteractor:
    def __init__(self) -> None:
        self.client = carla.Client(
            os.getenv("CARLA_HOST"), int(os.getenv("CARLA_PORT")))
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.graph, self.waypoints_locations = create_graph(
            self.world.get_map())

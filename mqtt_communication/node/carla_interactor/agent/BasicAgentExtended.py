import os
import sys
import carla
import numpy as np
import networkx as nx

carla_python_api_module_path = os.getenv('PYTHON_API_PATH')

if carla_python_api_module_path is None:
    raise ValueError("PYTHON_API_PATH environment variable not set.")

sys.path.append(carla_python_api_module_path)

try:
    from agents.navigation.basic_agent import BasicAgent 
except ImportError:
    raise ImportError("Cannot import the module. Check PYTHON_API_PATH and ensure it points to the 'carla' directory.")


class BasicAgentExtended(BasicAgent):
    def __init__(self, vehicle, graph ,map, road_id_to_edge, speed=30, final_point=None):
        super(BasicAgentExtended, self).__init__(vehicle, speed)
        self.vehicle = vehicle
        self._graph = graph
        self._map = map
        self._road_id_to_edge = road_id_to_edge
        self.final_point = None
        self.route = None

    def localize(self, x, y, z):
        location = carla.Location(x=x, y=y, z=z)
        waypoint = self._map.get_waypoint(location)
        edge = None
        try:
            edge = self._road_id_to_edge[waypoint.road_id][waypoint.section_id][waypoint.lane_id]
        except KeyError:
            pass
        return edge

    def distance_heuristic(self, n1, n2):
        l1 = np.array(self._graph.nodes[n1]['vertex'])
        l2 = np.array(self._graph.nodes[n2]['vertex'])
        return np.linalg.norm(l1-l2)

    def path_search(self, x1, y1, z1, x2, y2, z2):
        start, end = self.localize(x1, y1, z1), self.localize(x2, y2, z2)
        route = nx.astar_path(
            self._graph, source=start[0], target=end[0],
            heuristic=self.distance_heuristic, weight='weight')
        route.append(end[1])
        self.route = route
        self.plot_route(route)
        return route
    
    def update_weights_for_nodes(self, nodes_with_new_weights):
        for u, new_weight in nodes_with_new_weights.items():
            if u not in self._graph:
                raise ValueError(f"Node {u} not found in the graph.")
            
            for v in self._graph.neighbors(u):
                self._graph[u][v]['weight'] = new_weight

        return self._graph
    

    def plot_route(self, route):
        import matplotlib.pyplot as plt
        import numpy as np
        for i in range(len(route)-1):
            u = route[i]
            v = route[i+1]
            vertex1 = self._graph.nodes[u]['vertex']
            x1, y1 = vertex1[0], vertex1[1]
            vertex2 = self._graph.nodes[v]['vertex']
            x2, y2 = vertex2[0], vertex2[1]
            plt.plot([x1, x2], [-y1, -y2], 'ro-')
        plt.show()


    def update_plan(self, new_plan):
        self._local_planner.set_global_plan(new_plan)
        self.route = new_plan


    def run_step(self):
        control = super(BasicAgentExtended, self).run_step()

        return control
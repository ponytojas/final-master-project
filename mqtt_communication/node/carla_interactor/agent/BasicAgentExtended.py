import os
import sys
import carla
import numpy as np
import networkx as nx
from carla_interactor.agent.GlobalRoutePlannerExtended import GlobalRoutePlannerExtended

carla_python_api_module_path = os.getenv('PYTHON_API_PATH')

if carla_python_api_module_path is None:
    raise ValueError("PYTHON_API_PATH environment variable not set.")

sys.path.append(carla_python_api_module_path)

try:
    from agents.navigation.basic_agent import BasicAgent 
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    from agents.navigation.local_planner import RoadOption
except ImportError:
    raise ImportError("Cannot import the module. Check PYTHON_API_PATH and ensure it points to the 'carla' directory.")


class BasicAgentExtended(BasicAgent):
    def __init__(self, vehicle, graph ,map, road_id_to_edge, speed=30, final_point=None):
        super(BasicAgentExtended, self).__init__(vehicle, speed)
        self.vehicle = vehicle
        self._graph = graph
        self._map = map
        self._road_id_to_edge = road_id_to_edge
        self.initial_point = None
        self.final_point = None
        self.route = None
        self._global_planner = GlobalRoutePlanner(self._map, self._sampling_resolution)
        self._global_planner_extended = GlobalRoutePlannerExtended(self._map, self._sampling_resolution)

    def set_destination(self, end_location, start_location=None):
        if not start_location:
            start_location = self._local_planner.target_waypoint.transform.location
            clean_queue = True
        else:
            start_location = self._vehicle.get_location()
            clean_queue = False

        self.initial_point = self._map.get_waypoint(start_location)
        self.final_point = self._map.get_waypoint(end_location)

        self.path_search(
            self.initial_point.transform.location.x, self.initial_point.transform.location.y, self.initial_point.transform.location.z,
            self.final_point.transform.location.x, self.final_point.transform.location.y, self.final_point.transform.location.z
        )

        route_trace = self.update_plan()
        return route_trace


    def localize(self, x, y, z):
        location = carla.Location(x=x, y=y, z=z)
        waypoint = self._map.get_waypoint(location)
        edge = None
        try:
            edge = self._road_id_to_edge[waypoint.road_id][waypoint.section_id][waypoint.lane_id]
        except KeyError:
            pass
        if edge is None:
            # Find the closest edge
            min_distance = float('inf')
            for road_id in self._road_id_to_edge:
                for section_id in self._road_id_to_edge[road_id]:
                    for lane_id in self._road_id_to_edge[road_id][section_id]:
                        edge = self._road_id_to_edge[road_id][section_id][lane_id]
                        vertex = self._graph.nodes[edge]['vertex']
                        x1, y1 = vertex[0], vertex[1]
                        distance = np.linalg.norm(np.array([x, y]) - np.array([x1, y1]))
                        if distance < min_distance:
                            min_distance = distance
                            closest_edge = edge
            edge = closest_edge
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
        # self.plot_route(route)
        self.final_point = self._map.get_waypoint(carla.Location(x=x2, y=y2, z=z2))
        return route
    
    def update_weights_for_nodes(self, nodes_with_new_weights):
        # Iterate knowing nodes_with_new_weights is a list of tuples with node and new weight
        for u, new_weight in nodes_with_new_weights:
            if u not in self._graph:
                raise ValueError(f"Node {u} not found in the graph.")
            
            for v in self._graph.neighbors(u):
                self._graph[u][v]['weight'] = new_weight
    
        # Set initial point as the current vehicle location
        self.initial_point =  self._map.get_waypoint(self.vehicle.get_location())
        
        self.path_search(
            self.initial_point.transform.location.x, self.initial_point.transform.location.y, self.initial_point.transform.location.z,
            self.final_point.transform.location.x, self.final_point.transform.location.y, self.final_point.transform.location.z)
        
        self.update_plan()
        return self._graph

    def plot_route(self):
        import matplotlib.pyplot as plt
        import numpy as np

        plt.ion()  # Turn on interactive mode

        for i in range(len(self.route)-1):
            u = self.route[i]
            v = self.route[i+1]
            vertex1 = self._graph.nodes[u]['vertex']
            x1, y1 = vertex1[0], vertex1[1]
            vertex2 = self._graph.nodes[v]['vertex']
            x2, y2 = vertex2[0], vertex2[1]
            plt.plot([x1, x2], [-y1, -y2], 'ro-')
            plt.draw()  # Update the plot
            plt.pause(0.001)  # Pause briefly to allow the plot to update

        plt.ioff()  # Turn off interactive mode
        plt.show()  # Show the final plot


    def plot_route_wit_labels(self, route):
        import matplotlib.pyplot as plt
        
        plt.figure(figsize=(10, 8))  # Optional: Adjust the figure size
        
        for i in range(len(route) - 1):
            u = route[i]
            v = route[i + 1]
            
            vertex1 = self._graph.nodes[u]['vertex']
            x1, y1 = vertex1[0], vertex1[1]
            
            vertex2 = self._graph.nodes[v]['vertex']
            x2, y2 = vertex2[0], vertex2[1]
            
            plt.plot([x1, x2], [-y1, -y2], 'ro-')
            
            # Add labels for the nodes
            plt.text(x1, -y1, str(u), fontsize=12, ha='right')
            plt.text(x2, -y2, str(v), fontsize=12, ha='right')
        
        # Optional: Add labels for start and end points if route has more than one point
        if route:
            start = route[0]
            end = route[-1]
            start_vertex = self._graph.nodes[start]['vertex']
            end_vertex = self._graph.nodes[end]['vertex']
            plt.text(start_vertex[0], -start_vertex[1], f'Start ({start})', fontsize=12, ha='right', color='green')
            plt.text(end_vertex[0], -end_vertex[1], f'End ({end})', fontsize=12, ha='right', color='blue')
        
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Route Plot')
        plt.grid(True)
        plt.show()

    def update_plan(self):
        # self.plot_route()
        new_plan = self._global_planner_extended.trace_route_extended(self.route, self.initial_point, self.final_point)
        self._local_planner.set_global_plan(new_plan, clean_queue=True)
        self.route = new_plan

    def run_step(self):
        control = super(BasicAgentExtended, self).run_step()

        return control
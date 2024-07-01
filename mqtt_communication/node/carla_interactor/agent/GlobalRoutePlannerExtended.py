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
    from agents.navigation.global_route_planner import GlobalRoutePlanner 
    from agents.navigation.local_planner import RoadOption
except ImportError:
    raise ImportError("Cannot import the module. Check PYTHON_API_PATH and ensure it points to the 'carla' directory.")


class GlobalRoutePlannerExtended(GlobalRoutePlanner):
    def __init__(self, wmap, sampling_resolution):
        super(GlobalRoutePlannerExtended, self).__init__(wmap, sampling_resolution)

    def set_new_graph(self, graph):
        self._graph = graph

    def trace_route_extended(self, route, current_waypoint, destination_waypoint):
        """
        This method returns list of (carla.Waypoint, RoadOption)
        from origin to destination
        """
        route_trace = []
        destination = destination_waypoint.transform.location

        for i in range(len(route) - 1):
            road_option = self._turn_decision(i, route)
            edge = self._graph.edges[route[i], route[i+1]]
            path = []

            if edge['type'] != RoadOption.LANEFOLLOW and edge['type'] != RoadOption.VOID:
                route_trace.append((current_waypoint, road_option))
                exit_wp = edge['exit_waypoint']
                n1, n2 = self._road_id_to_edge[exit_wp.road_id][exit_wp.section_id][exit_wp.lane_id]
                next_edge = self._graph.edges[n1, n2]
                if next_edge['path']:
                    closest_index = self._find_closest_in_list(current_waypoint, next_edge['path'])
                    closest_index = min(len(next_edge['path'])-1, closest_index+5)
                    current_waypoint = next_edge['path'][closest_index]
                else:
                    current_waypoint = next_edge['exit_waypoint']
                route_trace.append((current_waypoint, road_option))

            else:
                path = path + [edge['entry_waypoint']] + edge['path'] + [edge['exit_waypoint']]
                closest_index = self._find_closest_in_list(current_waypoint, path)
                for waypoint in path[closest_index:]:
                    current_waypoint = waypoint
                    route_trace.append((current_waypoint, road_option))
                    if len(route)-i <= 2 and waypoint.transform.location.distance(destination) < 2*self._sampling_resolution:
                        break
                    elif len(route)-i <= 2 and current_waypoint.road_id == destination_waypoint.road_id and current_waypoint.section_id == destination_waypoint.section_id and current_waypoint.lane_id == destination_waypoint.lane_id:
                        destination_index = self._find_closest_in_list(destination_waypoint, path)
                        if closest_index > destination_index:
                            break

        return route_trace
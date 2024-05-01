import os
import carla
import networkx as nx

class Router:
    def __init__(self):
        # Connect to the CARLA server
        carla_host = os.getenv('CARLA_HOST') or "localhost"
        carla_port = int(os.getenv('CARLA_PORT') or 2000)
        
        self.client = carla.Client(carla_host, carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        
        # Create a graph representation of the road network
        self.graph = self.create_road_network_graph()
        
    def create_road_network_graph(self):
        """
        Creates a graph representation of the road network in CARLA.
        """
        graph = nx.Graph()
        for waypoint in self.map.generate_waypoints(distance=1):
            # Add a node for each waypoint, with its position
            graph.add_node(waypoint.id, pos=(waypoint.transform.location.x, waypoint.transform.location.y))
            
            # For each waypoint, connect it to the next waypoints to form the graph edges
            for next_waypoint in waypoint.next(1):
                # The weight could be the distance between waypoints
                weight = waypoint.transform.location.distance(next_waypoint.transform.location)
                graph.add_edge(waypoint.id, next_waypoint.id, weight=weight)
        
        return graph

    def find_closest_waypoint(self, location):
        """
        Finds the closest waypoint to a given location that is present in the graph.
        
        Args:
            location (carla.Location): The location to find the closest waypoint to.
        
        Returns:
            carla.Waypoint: The closest waypoint to the specified location that is present in the graph.
        """
        closest_waypoint = None
        min_distance = float('inf')
        
        for waypoint in self.map.generate_waypoints(distance=1):
            distance = location.distance(waypoint.transform.location)
            if distance < min_distance and waypoint.id in self.graph:
                closest_waypoint = waypoint
                min_distance = distance
        
        return closest_waypoint

    def calculate_route(self, start_location, end_location):
        """
        Calculates the shortest route from start_location to end_location using A* algorithm.
        
        Args:
            start_location (carla.Location): The starting point of the route.
            end_location (carla.Location): The destination point of the route.
        
        Returns:
            List[carla.Location]: The calculated route as a list of carla.Location objects.
        """
        # Find the closest waypoints to the start and end locations
        start_waypoint = self.find_closest_waypoint(start_location)
        end_waypoint = self.find_closest_waypoint(end_location)
        
        if start_waypoint is None or end_waypoint is None:
            print("No valid waypoints found for the specified start or end location.")
            return []
        
        # Use NetworkX's A* algorithm to find the shortest path between waypoints
        try:
            path = nx.astar_path(self.graph, start_waypoint.id, end_waypoint.id, weight='weight')
        except nx.NetworkXNoPath as e:
            print("No path found between the specified points.")
            print(e)
            return []
        
        # Convert the path of waypoint IDs back to locations
        route = [self.map.get_waypoint_by_id(wp_id).transform.location for wp_id in path]
        
        return route

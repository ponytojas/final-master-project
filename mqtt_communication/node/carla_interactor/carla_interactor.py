import os
import sys
import time
import carla
import carla.libcarla as lc
import numpy as np
import random
import logging
from math import sqrt, pi, cos, sin, atan2, radians

from mqtt_node.mqtt_client import MQTTClient
from mqtt_node.data_manager import DataManager
from carla_interactor.spawn_actor import spawn_actor
from carla_interactor.create_planner import create_planner
from carla_interactor.graph.utils import find_shortest_path, plot_graph, add_weights_to_edges, path_search
from carla_interactor.control import ControlPose, TrackingController
from carla_interactor.agent.BasicAgentExtended import BasicAgentExtended
# from carla.agents.navigation import GlobalRoutePlanner

carla_python_api_module_path = os.getenv('PYTHON_API_PATH')

if carla_python_api_module_path is None:
    raise ValueError("PYTHON_API_PATH environment variable not set.")

sys.path.append(carla_python_api_module_path)

try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner 
    from agents.navigation.basic_agent import BasicAgent 

except ImportError:
    raise ImportError("Cannot import the module. Check PYTHON_API_PATH and ensure it points to the 'carla' directory.")



START_LOCATION = {'x': -103.092, 'y': -13.207, 'z': 0.599}
END_LOCATION = {'x': -27.727, 'y': -68.283, 'z': 0.0}



class CarlaInteractor:
    # def __init__(self, type=None):
    def __init__(self, type='test'):
        self.client_id = str(random.randint(0, 4294967295))
        logging.info(f"Client ID: {self.client_id}")

        # self.actor_type = type if type is not None else random.choice(['vehicle', 'walker'])
        self.actor_type = type or 'vehicle'

        self.mqtt_client = MQTTClient(os.getenv("MQTT_HOST"), int(os.getenv("MQTT_PORT")), self.client_id)
        self.mqtt_client.connect()
        logging.debug("Connected to MQTT broker")
        self.mqtt_client.subscribe(self.actor_type)

        self.data_manager = DataManager(self.mqtt_client, self.actor_type)
        logging.debug("DataManager initialized")

        self.mqtt_client.set_data_manager(self.data_manager)

        self.carla_client = carla.Client(os.getenv("CARLA_HOST"), int(os.getenv("CARLA_PORT")))
        world = self.carla_client.get_world()
        self.map = world.get_map()

        self.GRP = GlobalRoutePlannerExtended(self.map, 0.25)
        self.graph = add_weights_to_edges(self.GRP._graph)

        # plot_graph(self.graph)
 
        self.actor, self.ai_controller, self.world = spawn_actor(self.actor_type)


        self.agent = BasicAgentExtended(self.actor, self.graph, self.map, self.GRP._road_id_to_edge)
        # self.agent = BasicAgent(self.actor, 60)

        # Get actor current position
        spawn_points = world.get_map().get_spawn_points()
        initial_point = spawn_points[2]
        self.initial_point = carla.Location(initial_point.location.x, initial_point.location.y, initial_point.location.z)

        self.final_point = carla.Location(END_LOCATION['x'], END_LOCATION['y'], END_LOCATION['z'])
        self.current_point = self.initial_point

        self.agent.path_search(self.initial_point.x, self.initial_point.y, self.initial_point.z, self.final_point.x, self.final_point.y, self.final_point.z)
        self.agent.set_destination(self.final_point)


    def adjust_graph_weights(self, waypoints):
        for waypoint in waypoints:
            self.graph.nodes[waypoint]['weight'] = sys.maxint
        

    def generate_position_message(self):
        new_data = {
                "client_id": self.client_id,
                "location": {
                    "x": self.actor.get_location().x,
                    "y": self.actor.get_location().y,
                    "z": self.actor.get_location().z
                },
                "velocity": {
                    "x": self.actor.get_velocity().x,
                    "y": self.actor.get_velocity().y,
                    "z": self.actor.get_velocity().z
                },
                "acceleration": {
                    "x": self.actor.get_acceleration().x,
                    "y": self.actor.get_acceleration().y,
                    "z": self.actor.get_acceleration().z
                },
                "heading": self.actor.get_transform().rotation.yaw
            }
        self.data_manager.update_data(new_data)
        self.world.tick()

    def follow_route(self):
        self.actor.apply_control(self.agent.run_step())

    def destroy(self):
        if self.actor:
            self.actor.destroy()
            logging.debug("Actor destroyed.")
        if self.ai_controller:
            self.ai_controller.stop()
            self.ai_controller.destroy()
            logging.debug("AI Controller destroyed.")
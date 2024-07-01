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


    # def follow_route(self):
    #     # route = find_shortest_path(self.graph, self.initial_point_id, self.final_point_id)
    #     route = self.GRP.trace_route(self.initial_point, self.final_point)

    #     MIN_DIST = 0.2
    #     spline_waypoints_route = []
    #     v_road = []
    #     aux = []
        
    #     st_time = time.time()
    #     prev_waypoint = route[0][0]
    #     print("FIRST WAYPOINT: ", prev_waypoint.transform.location.x, prev_waypoint.transform.location.y, prev_waypoint.transform.location.z)
    #     print("LAST WAYPOINT: ", route[-1][0].transform.location.x, route[-1][0].transform.location.y, route[-1][0].transform.location.z)

    #     for i in range(1, len(route)):
    #         waypoint = route[i][0]
    #         aux.append(waypoint)
    #         # Calculate distance between waypoints
    #         dist = sqrt(pow(waypoint.transform.location.x - prev_waypoint.transform.location.x, 2) + pow(waypoint.transform.location.y - prev_waypoint.transform.location.y, 2))
    #         # dist = sqrt(pow(waypoint['x'] - prev_waypoint['x'], 2) + pow(waypoint['y'] - prev_waypoint['y'], 2))

    #         if dist > MIN_DIST:
    #             aux_pose = ControlPose(waypoint.transform.location.x, -waypoint.transform.location.y, waypoint.transform.location.z, radians(waypoint.transform.rotation.yaw))
    #             # aux_pose = ControlPose(waypoint['x'], -waypoint['y'], waypoint['z'], radians(waypoint['transform'].rotation.yaw))
    #             spline_waypoints_route.append(aux_pose)
    #             v_road.append(14)
    #             prev_waypoint = waypoint

    #     self.tracking_controller.trajectory_spline_interpolation(spline_waypoints_route)

    #     sp_list = []
    #     for i, coef in enumerate(self.tracking_controller.spline_coeffs):
    #         u_values = np.arange(0.1, 1.1, 0.1)
    #         x_aux = coef[0] + coef[1]*u_values + coef[2]*np.power(u_values, 2) + coef[3]*np.power(u_values, 3)
    #         y_aux = coef[4] + coef[5]*u_values + coef[6]*np.power(u_values, 2) + coef[7]*np.power(u_values, 3)
    #         sp_list.extend(np.column_stack((x_aux, y_aux)))
        
        
    #     V_MAX = 7
    #     self.tracking_controller.velocity_profile_generation(v_road, V_MAX)
    #     self.tracking_controller.stop_requested = False

    #     transform = self.actor.get_transform()
    #     vehicle_control_pose = ControlPose( transform.location.x, \
    #                                         transform.location.y, \
    #                                         transform.location.z, \
    #                                         radians(transform.rotation.yaw))
    #     vehicle_control_pose.u = 0
    #     vehicle_control_pose.tramo = 0
    #     self.tracking_controller.car_pose = vehicle_control_pose

    #     fi_time = time.time()
    #     logging.debug(f"Time to generate trajectory: {fi_time - st_time}")

    #     done = False
    #     KP = 0.15
    #     KI = 0.001
    #     ERROR_SUM = 0
    #     t0 = time.time()

    #     while not done:
    #         t_now = time.time() - t0
            
    #         # Update spectator position
    #         yaw_car         = self.actor.get_transform().rotation.yaw
    #         x_spectator     = self.actor.get_transform().location.x - 10*cos(radians(yaw_car))
    #         y_spectator     = self.actor.get_transform().location.y - 10*sin(radians(yaw_car))
    #         world_transform = carla.Transform(carla.Location(x=x_spectator, y=y_spectator, z=10), carla.Rotation(yaw=yaw_car, pitch = -30))
    #         self.world.get_spectator().set_transform(world_transform) 
            
    #         transform = self.actor.get_transform()
    #         self.tracking_controller.car_pose.x = transform.location.x
    #         self.tracking_controller.car_pose.y = transform.location.y
    #         self.tracking_controller.car_pose.z = transform.location.z
    #         self.tracking_controller.car_pose.o = radians(transform.rotation.yaw)


    #         # Velocidad actual
    #         actual_speed = sqrt(self.actor.get_velocity().x**2 + \
    #                     self.actor.get_velocity().y**2)
            
    #         # Control lateral
    #         self.tracking_controller.control_move()
    #         Ulqr = self.tracking_controller.cmd_vel.w
    #         # Ulqr = -Ulqr

    #         # Control longitudinal
    #         v_ref = self.tracking_controller.cmd_vel.v

    #         error = v_ref - actual_speed
    #         ERROR_SUM += error

    #         PIDbrake = 0
    #         PIDthrottle = KP*error + KI*ERROR_SUM
    #         if PIDthrottle > 1:
    #             PIDthrottle = 1
    #         elif PIDthrottle < 0:
    #             PIDthrottle = 0

    #         if v_ref == 0:
    #             PIDbrake = 1
    #             PIDthrottle = 0
    #             ERROR_SUM = 0
            
    #         # Aplicar control
    #         self.actor.apply_control(carla.VehicleControl(throttle=PIDthrottle, \
    #                                                     steer=-Ulqr, \
    #                                                     brake=PIDbrake))


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
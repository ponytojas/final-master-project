import logging
import random
import carla


def create_vehicle(carla_host: str, carla_port: int, test=False):
    logging.debug(f"Connecting to Carla at {carla_host}:{carla_port}")
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(10.0)
    logging.debug("Connected to Carla")
    world = client.get_world()
    spawn_points = world.get_map().get_spawn_points()

    if test:
        initial_point = spawn_points[2]
        logging.debug(f"Selected spawn point: {initial_point}")
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.audi.a2')[0]
        actor = world.spawn_actor(vehicle_bp, initial_point)
    else:
        blueprint_library = world.get_blueprint_library()
        vehicles_blueprint = blueprint_library.filter("vehicle.*")
        vehicle_bp = blueprint_library.filter('vehicle.audi.a2')[0]
        logging.debug(f"Selected blueprint: {vehicle_bp.id}")
        spawn_points = world.get_map().get_spawn_points()
        initial_point = spawn_points[2]
        logging.debug(f"Selected spawn point: {initial_point}")
        actor = world.spawn_actor(vehicle_bp, initial_point)

    # actor.set_autopilot(True)
    logging.debug("Vehicle spawned with autopilot")
    return actor, None, world

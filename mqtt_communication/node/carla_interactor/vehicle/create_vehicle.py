import logging
import random
import carla


def create_vehicle(carla_host: str, carla_port: int):
    logging.debug(f"Connecting to Carla at {carla_host}:{carla_port}")
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(10.0)
    logging.debug("Connected to Carla")
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    vehicles_blueprint = blueprint_library.filter("vehicle.*")
    actor_bp = random.choice(vehicles_blueprint)
    logging.debug(f"Selected blueprint: {actor_bp.id}")
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = random.choice(spawn_points)
    actor = world.spawn_actor(actor_bp, spawn_point)
    actor.set_autopilot(True)
    logging.debug("Vehicle spawned with autopilot")
    return actor, None, world

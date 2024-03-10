import carla
import time
import logging
import random

PERCENTAGE_PEDESTRIANS_RUNNING = 0.3


def create_walker(carla_host: str, carla_port: int):
    seed = int(time.time())
    random.seed(seed)
    logging.debug(f"Connecting to Carla at {carla_host}:{carla_port}")
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(10.0)
    logging.debug("Connected to Carla")
    world = client.get_world()
    world.set_pedestrians_seed(seed)
    random_int_value = random.randint(10, 100)
    for i in range(random_int_value):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()

    if (loc != None):
        spawn_point.location = loc
    else:
        logging.error("No valid spawn point found")
        return None

    walker_bp = random.choice(world.get_blueprint_library().filter("walker.*"))
    walker_speed = 0.0
    if walker_bp.has_attribute('is_invincible'):
        walker_bp.set_attribute('is_invincible', 'false')
    if walker_bp.has_attribute('speed'):
        if (random.random() > PERCENTAGE_PEDESTRIANS_RUNNING):
            # walking
            walker_speed = walker_bp.get_attribute(
                'speed').recommended_values[1]
        else:
            # running
            walker_speed = walker_bp.get_attribute(
                'speed').recommended_values[2]
    else:
        logging.info("Walker has no speed")
        walker_speed = 0.0

    walker = world.spawn_actor(walker_bp, spawn_point)
    logging.debug("Walker spawned successfully")
    world.tick()

    ai_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    ai_controller = world.spawn_actor(
        ai_controller_bp, carla.Transform(), walker)

    destination = world.get_random_location_from_navigation()
    ai_controller.start()
    ai_controller.go_to_location(destination)
    ai_controller.set_max_speed(float(walker_speed))

    logging.debug("Walker AI controller started")

    return walker, ai_controller, world

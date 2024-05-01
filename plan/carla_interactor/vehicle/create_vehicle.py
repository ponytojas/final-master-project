import logging
import random
import carla

def create_vehicle(carla_host: str, carla_port: int, start_location=None, end_location=None):
    logging.debug(f"Connecting to Carla at {carla_host}:{carla_port}")
    client = carla.Client(carla_host, carla_port)
    client.set_timeout(10.0)
    logging.debug("Connected to Carla")
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    vehicles_blueprint = blueprint_library.filter("vehicle.*")
    actor_bp = random.choice(vehicles_blueprint)
    logging.debug(f"Selected blueprint: {actor_bp.id}")

    # Use CARLA's built-in method to find spawn points
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        logging.error("No spawn points were found on the map.")
        return None
    

    # start_location = carla.Transform(carla.Location(x=-104.7, y=-66.9, z=0.0))
    # end_location = carla.Transform(carla.Location(x=-153.5, y=154.5, z=0.0))

    start_location = carla.Transform(carla.Location(x=-153.5, y=154.5, z=0.0))
    end_location = carla.Transform(carla.Location(x=-104.7, y=-66.9, z=0.0))


    if start_location is None:
        # Optionally find a random spawn point that's unoccupied
        start_location = random.choice(spawn_points)
    else:
        # If a start location is specified, find the nearest spawn point to the specified location
        start_location = min(spawn_points, key=lambda p: p.location.distance(start_location.location))

    logging.debug(f'''Start location: {start_location.location.x}, {start_location.location.y}, {start_location.location.z}''')
    logging.debug(f'''End location: {end_location.location.x}, {end_location.location.y}, {end_location.location.z}''')

    actor = None
    try:
        actor = world.spawn_actor(actor_bp, start_location)
        actor.set_autopilot(False)  # Disable autopilot
        logging.debug("Vehicle spawned without autopilot")
    except Exception as e:
        logging.error(f"Failed to spawn vehicle: {e}")
        return None
    return actor, start_location, end_location, world

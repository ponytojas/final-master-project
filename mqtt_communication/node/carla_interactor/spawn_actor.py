from .vehicle import create_vehicle
from .walker import create_walker
import os


def spawn_actor(actor_type: str):
    carla_host = os.getenv('CARLA_HOST') or "localhost"
    carla_port = int(os.getenv('CARLA_PORT') or 2000)

    if actor_type == 'vehicle':
        return create_vehicle(carla_host=carla_host, carla_port=carla_port)
    else:
        return create_walker(carla_host=carla_host, carla_port=carla_port)

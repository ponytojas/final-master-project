import asyncio
import os
import carla
import logging
import random

from carla_interactor.spawn_actor import spawn_actor
from carla_interactor.router.router import Router
from dotenv import load_dotenv
from mqtt_node.mqtt_client import MQTTClient
from mqtt_node.data_manager import DataManager

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')

load_dotenv()

START_LOCATION = {'x':-104.69999694824219, 'y': -66.9000015258789, 'z':0.0}
END_LOCATION = {'x':-135.2351837158203, 'y':147.57672119140625, 'z':0.5}


async def carla_simulation():
    client_id = str(random.randint(0, 4294967295))
    logging.info(f"Client ID: {client_id}")

    actor_type = 'vehicle'

    mqtt_client = MQTTClient(os.getenv("MQTT_HOST"), int(
        os.getenv("MQTT_PORT")), client_id)

    mqtt_client.connect()
    logging.debug("Connected to MQTT broker")
    mqtt_client.subscribe(actor_type)

    data_manager = DataManager(mqtt_client, actor_type)
    logging.debug("DataManager initialized")

    mqtt_client.set_data_manager(data_manager)

    router = Router()
    start_location = carla.Location(x=START_LOCATION['x'], y=START_LOCATION['y'], z=START_LOCATION['z'])
    end_location = carla.Location(x=END_LOCATION['x'], y=END_LOCATION['y'], z=END_LOCATION['z'])

    # Calculate the route
    route = router.calculate_route(start_location, end_location)
    logging.debug(f"Route: {route}")


    actor = None
    ai_controller = None
    world = None
    try:
        actor, start_location, end_location, world = spawn_actor(actor_type)

        while True:
            new_data = {
                "client_id": client_id,
                "location": {
                    "x": actor.get_location().x,
                    "y": actor.get_location().y,
                    "z": actor.get_location().z
                },
                "velocity": {
                    "x": actor.get_velocity().x,
                    "y": actor.get_velocity().y,
                    "z": actor.get_velocity().z
                },
                "acceleration": {
                    "x": actor.get_acceleration().x,
                    "y": actor.get_acceleration().y,
                    "z": actor.get_acceleration().z
                },
                "heading": actor.get_transform().rotation.yaw
            }
            data_manager.update_data(new_data)
            world.tick()
            await asyncio.sleep(1)
    except Exception as e:
        logging.error(f"Error: {e}")
    finally:
        if actor:
            actor.destroy()
            logging.debug("Actor destroyed.")
        if ai_controller:
            ai_controller.stop()
            ai_controller.destroy()
            logging.debug("AI Controller destroyed.")


async def main():
    logging.debug("Starting main")
    await asyncio.gather(carla_simulation())

if __name__ == "__main__":
    asyncio.run(main())

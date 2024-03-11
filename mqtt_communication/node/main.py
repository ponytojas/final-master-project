import asyncio
import os
import argparse
import logging
import random

from carla_interactor.spawn_actor import spawn_actor
from dotenv import load_dotenv
from mqtt_node.mqtt_client import MQTTClient
from mqtt_node.data_manager import DataManager

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')

load_dotenv()


async def carla_simulation(type: str = None):
    client_id = str(random.randint(0, 4294967295))
    logging.info(f"Client ID: {client_id}")

    actor_type = type if type is not None else random.choice(
        ['vehicle', 'walker'])

    mqtt_client = MQTTClient(os.getenv("MQTT_HOST"), int(
        os.getenv("MQTT_PORT")), client_id)

    mqtt_client.connect()
    logging.debug("Connected to MQTT broker")
    mqtt_client.subscribe(actor_type)

    data_manager = DataManager(mqtt_client, actor_type)
    logging.debug("DataManager initialized")

    mqtt_client.set_data_manager(data_manager)

    actor = None
    ai_controller = None
    world = None
    try:
        actor, ai_controller, world = spawn_actor(actor_type)

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


async def main(type: str = None):
    logging.debug("Starting main")
    await asyncio.gather(carla_simulation(type))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Your program description')
    parser.add_argument('-t', '--type', type=str, choices=['vehicle', 'walker'], default=None,
                        help='Specify the type (vehicle or walker)')

    args = parser.parse_args()

    asyncio.run(main(args.type))
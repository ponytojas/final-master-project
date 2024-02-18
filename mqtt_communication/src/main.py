import asyncio
import os
import uuid
import random
import logging
import carla
from dotenv import load_dotenv
from mqtt_node.mqtt_client import MQTTClient
from mqtt_node.data_manager import DataManager

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')

load_dotenv()

MQTT_HOST = os.getenv("MQTT_HOST") or "localhost"
MQTT_PORT = int(os.getenv("MQTT_PORT") or 1883)
MQTT_TOPIC = os.getenv("MQTT_TOPIC") or "ros/data"
CARLA_HOST = os.getenv('CARLA_HOST') or "localhost"
CARLA_PORT = int(os.getenv('CARLA_PORT')) or 2000


async def carla_simulation(client_id: str, data_manager: DataManager):
    global CARLA_HOST, CARLA_PORT
    try:
        logging.debug(f"Connecting to Carla at {CARLA_HOST}:{CARLA_PORT}")
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        client.set_timeout(10.0)
        logging.debug("Connected to Carla")
        world = client.get_world()

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        logging.debug(f"Selected vehicle blueprint: {vehicle_bp.id}")

        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        vehicle.set_autopilot(True)
        logging.debug("Vehicle spawned")

        while True:
            new_data = {
                "client_id": client_id,
                "location": {"x": vehicle.get_location().x, "y": vehicle.get_location().y, "z": vehicle.get_location().z},
                "rotation": {"yaw": vehicle.get_transform().rotation.yaw, "pitch": vehicle.get_transform().rotation.pitch, "roll": vehicle.get_transform().rotation.roll},
                "acceleration": {"x": vehicle.get_acceleration().x, "y": vehicle.get_acceleration().y, "z": vehicle.get_acceleration().z},
                "velocity": {"x": vehicle.get_velocity().x, "y": vehicle.get_velocity().y, "z": vehicle.get_velocity().z},
                "angular_velocity": {"x": vehicle.get_angular_velocity().x, "y": vehicle.get_angular_velocity().y, "z": vehicle.get_angular_velocity().z}
            }
            data_manager.update_data(new_data)
            await asyncio.sleep(1)

    finally:
        if vehicle:
            vehicle.destroy()
            print("Vehicle destroyed.")


async def main():
    global MQTT_HOST, MQTT_PORT, MQTT_TOPIC

    logging.debug("Starting main")
    client_id = str(uuid.uuid4())
    logging.info(f"Client ID: {client_id}")

    mqtt_client = MQTTClient(MQTT_HOST, MQTT_PORT, client_id)
    mqtt_client.connect()
    logging.debug("Connected to MQTT broker")
    mqtt_client.subscribe(MQTT_TOPIC)
    logging.debug(f"Subscribed to topic {MQTT_TOPIC}")

    data_manager = DataManager(mqtt_client)
    logging.debug("DataManager initialized")

    await asyncio.gather(
        carla_simulation(client_id, data_manager),
    )

if __name__ == "__main__":
    asyncio.run(main())

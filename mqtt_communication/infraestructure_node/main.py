import asyncio
import os
import logging
import carla

from dotenv import load_dotenv
from node.InfrastructureNode import InfrastructureNode

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')

load_dotenv()


async def carla_simulation():

    logging.debug("Starting carla_simulation")

    logging.debug("Carla simulation started")

    node = InfrastructureNode(os.getenv("MQTT_HOST"),
                              int(os.getenv("MQTT_PORT")), "INFRAESTRUCTURE")
    node.connect()
    while True:
        await asyncio.sleep(1)


async def main():
    logging.debug("Starting main")
    await asyncio.gather(carla_simulation())

if __name__ == "__main__":
    asyncio.run(main())

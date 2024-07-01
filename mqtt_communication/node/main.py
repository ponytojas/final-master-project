import asyncio
import argparse
import logging

from dotenv import load_dotenv
from carla_interactor.carla_interactor import CarlaInteractor

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')

load_dotenv()


async def carla_simulation(type: str = None):
    interactor = CarlaInteractor(type)
    try:
        while True:
            interactor.follow_route()
            interactor.generate_position_message()
            interactor.check_for_denm()
    except Exception as e:
        logging.error(f"Error: {e}")
    finally:
        interactor.destroy()


async def main(type: str = None):
    logging.debug("Starting main")
    await asyncio.gather(carla_simulation(type))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Select the type of actor to spawn in the simulation.')
    parser.add_argument('-t', '--type', type=str, choices=['vehicle', 'walker'], default=None,
                        help='Specify the type (vehicle or walker)')

    args = parser.parse_args()

    asyncio.run(main(args.type))

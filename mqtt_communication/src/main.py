import os
import time
import uuid
from dotenv import load_dotenv
from mqtt_node import DataManager, MQTTClient

load_dotenv()

MQTT_HOST = os.getenv("MQTT_HOST") or "mqtt"
MQTT_PORT = int(os.getenv("MQTT_PORT") or 1883)


def main():
    broker_address = MQTT_HOST
    port = MQTT_PORT
    client_id = str(uuid.uuid4())

    mqtt_client = MQTTClient(broker_address, port, client_id)
    mqtt_client.connect()

    data_manager = DataManager(mqtt_client)
    counter = 0
    while True:
        new_data = {"client_id": client_id, "propiedad3": counter}
        counter += 1
        data_manager.update_data(new_data)
        time.sleep(5)


if __name__ == "__main__":
    main()

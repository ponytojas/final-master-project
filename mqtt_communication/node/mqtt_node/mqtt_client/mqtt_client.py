import logging
from typing import List, Tuple, Dict
import paho.mqtt.client as mqtt

SUSCRIPTIONS = {
    "vehicle": [
        "v2x/vehicle/+/data",
        "v2x/infrastructure/+"
    ],
    "walker": [
        "v2x/infrastructure/+"
    ]
}


class MQTTClient:
    def __init__(self, host: str, port: int, client_id: str):
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self.host = host
        self.port = port
        self.received_messages: List[Tuple[str, Dict]] = []
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client_id = client_id
        self.data_manager = None

    def set_data_manager(self, data_manager):
        self.data_manager = data_manager

    def on_connect(self, client, userdata, flags, rc, properties):
        logging.debug(f"Connected with result code {rc}")

    def on_message(self, client, userdata, message):
        if self.data_manager is None:
            return
        topic = message.topic
        schema = topic.split("/")[-1]
        decoded_message = self.data_manager.decode_message(
            message.payload, schema)
        
        if schema == 'denm':
            self.data_manager.set_denm_messages(decoded_message)
            return

        if str(decoded_message['header']['stationID']) == self.client_id:
            return
        self.received_messages.append((message.topic, decoded_message))
        logging.debug(f"Received message: {message.topic}")

    def connect(self):
        self.client.connect(self.host, self.port)
        self.client.loop_start()

    def subscribe(self, actor_type: str):
        if actor_type not in SUSCRIPTIONS:
            return
        for topic in SUSCRIPTIONS[actor_type]:
            self.client.subscribe(topic)

    def publish(self, topic: str, data: bytes):
        self.client.publish(topic, data)

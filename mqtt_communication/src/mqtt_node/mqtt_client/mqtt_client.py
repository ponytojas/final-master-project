import json
import logging
import paho.mqtt.client as mqtt


class MQTTClient:
    def __init__(self, host, port, client_id):
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self.host = host
        self.port = port
        self.received_messages = []
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client_id = client_id

    def on_connect(self, client, userdata, flags, rc, properties):
        logging.debug(f"Connected with result code {rc}")

    def on_message(self, client, userdata, message):
        data = json.loads(message.payload.decode('utf-8'))
        if (data['client_id'] == self.client_id):
            return
        self.received_messages.append((message.topic, data))
        logging.debug(f"Received message: {message.topic}")

    def connect(self):
        self.client.connect(self.host, self.port)
        self.client.loop_start()

    def subscribe(self, topic):
        self.client.subscribe(topic)

    def publish(self, topic, data):
        self.client.publish(topic, json.dumps(data))

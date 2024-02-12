import paho.mqtt.client as mqtt
import json


class MQTTClient:
    def __init__(self, host, port, client_id):
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self.host = host
        self.port = port
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc, properties):
        print(f"Conectado con código de resultado: {rc}")

    def on_message(self, client, userdata, message):
        print(
            f"Mensaje recibido: {message.payload.decode('utf-8')} en el topic {message.topic}")

    def connect(self):
        self.client.connect(self.host, self.port)
        self.client.loop_start()

    def publish(self, topic, data):
        self.client.publish(topic, json.dumps(data))

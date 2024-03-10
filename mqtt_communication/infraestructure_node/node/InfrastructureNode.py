import logging
import paho.mqtt.client as mqtt
import asn1tools
from typing import Dict


class InfrastructureNode:
    def __init__(self, host: str, port: int, client_id: str):
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self.host = host
        self.port = port
        self.client_id = client_id
        self.cam_asn = asn1tools.compile_files("cam_schema_carla.asn", "uper")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc, properties):
        logging.debug(f"Connected with result code {rc}")
        self.client.subscribe("v2x/vehicle/+/data")

    def on_message(self, client, userdata, message):
        cam_data = self.decode_cam_message(message.payload)
        logging.debug(
            f"Received CAM message from {cam_data['header']['stationID']}")
        self.resend_cam_message(message.payload)

    def connect(self):
        self.client.connect(self.host, self.port)
        self.client.loop_start()

    def decode_cam_message(self, encoded_data: bytes) -> Dict:
        decoded_data = self.cam_asn.decode("CAM", encoded_data)
        return decoded_data

    def resend_cam_message(self, encoded_data: bytes):
        self.client.publish("v2x/infrastructure/cam", encoded_data)

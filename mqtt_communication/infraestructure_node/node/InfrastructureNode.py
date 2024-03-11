import logging
import os
import paho.mqtt.client as mqtt
import asn1tools
from typing import Dict


def compile_asn_files(folder_path):
    schema_dict = {}

    for file_name in os.listdir(folder_path):
        if file_name.endswith('_schema.asn') or file_name.endswith('_carla.asn'):
            if file_name == 'cam_schema.asn' and 'cam_schema_carla.asn' in os.listdir(folder_path):
                continue
            key = file_name.split('_schema.asn')[0] if file_name.endswith(
                '_schema.asn') else file_name.split('_schema_carla.asn')[0]
            file_path = os.path.join(folder_path, file_name)
            schema_dict[key] = asn1tools.compile_files(file_path, "uper")

    logging.debug(f"ASN.1 schema files compiled: {schema_dict.keys()}")
    return schema_dict


SUSCRIPTIONS = [
    "v2x/vehicle/#",
    "v2x/walker/#",
]


class InfrastructureNode:
    def __init__(self, host: str, port: int, client_id: str):
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self.host = host
        self.port = port
        self.client_id = client_id
        self.asn_parsers = compile_asn_files('./asn_files/')
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc, properties):
        logging.debug(f"Connected with result code {rc}")
        for topic in SUSCRIPTIONS:
            self.client.subscribe(topic)

    def on_message(self, client, userdata, message):
        topic = message.topic
        schema = topic.split("/")[-1]

        if self.asn_parsers.get(schema) is None:
            logging.debug(f"Schema {schema} not found")
            return

        decoded_data = self.decode_message(message.payload, schema)
        logging.debug(
            f"Received CAM message from {decoded_data['header']['stationID']}")

        if (schema == "cam"):
            self.resend_cam_message(message.payload)
            return

    def connect(self):
        self.client.connect(self.host, self.port)
        self.client.loop_start()

    def decode_message(self, encoded_data: bytes, schema_name: str) -> Dict:
        decoded_data = self.asn_parsers[schema_name].decode(
            schema_name.upper(), encoded_data)
        return decoded_data

    def resend_cam_message(self, encoded_data: bytes):
        self.client.publish("v2x/infrastructure/cam", encoded_data)

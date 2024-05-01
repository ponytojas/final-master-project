import os
import logging
from typing import Dict
from mqtt_node.mqtt_client import MQTTClient
import asn1tools


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


class DataManager:
    def __init__(self, mqtt_client: MQTTClient, actor_type: str):
        self.mqtt_client = mqtt_client
        self.data: Dict = {}
        self.actor_type = actor_type
        self.topic = f"v2x/{actor_type}/{self.mqtt_client.client_id}/cam"
        self.stationType = 5 if actor_type == "vehicle" else 1
        self.asn_parsers = compile_asn_files('./asn_files/')

    def update_data(self, new_data: Dict):
        if self._data_deep_compare(new_data):
            return
        encoded_data = self.encode_cam_message(new_data)
        self.data.update(new_data)
        self.mqtt_client.publish(self.topic, encoded_data)

    def encode_cam_message(self, data: Dict) -> bytes:
        cam_data = {
            "header": {
                "protocolVersion": 1,
                "messageID": 1,
                "stationID": int(data["client_id"].replace("-", ""), 10)
            },
            "cam": {
                "generationDeltaTime": 0,
                "camParameters": {
                    "basicContainer": {
                        "stationType": self.stationType,
                        "referencePosition": {
                            "latitude": data["location"]["x"],
                            "longitude": data["location"]["y"],
                            "positionConfidenceEllipse": {
                                "semiMajorConfidence": 100,
                                "semiMinorConfidence": 50,
                                "semiMajorOrientation": 0
                            },
                            "altitude": {
                                "altitudeValue": data["location"]["z"],
                                "altitudeConfidence": 1
                            }
                        }
                    }
                }
            }
        }
        encoded_data = self.asn_parsers['cam'].encode('CAM', cam_data)
        return encoded_data

    def _data_deep_compare(self, new_data: Dict) -> bool:
        return all(self.data.get(key) == value for key, value in new_data.items())

    def decode_message(self, encoded_data: bytes, schema_name: str) -> Dict:
        if self.asn_parsers.get(schema_name) is None:
            logging.debug(f"Schema {schema_name} not found")
            return
        decoded_data = self.asn_parsers[schema_name].decode(
            schema_name.upper(), encoded_data)
        return decoded_data

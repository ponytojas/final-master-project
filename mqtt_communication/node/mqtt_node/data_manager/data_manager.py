import time
from typing import Dict
from mqtt_node.mqtt_client import MQTTClient
import asn1tools


class DataManager:
    def __init__(self, mqtt_client: MQTTClient, actor_type: str):
        self.mqtt_client = mqtt_client
        self.data: Dict = {}
        self.actor_type = actor_type
        self.topic = f"v2x/{actor_type}/{self.mqtt_client.client_id}/data"
        self.stationType = 5 if actor_type == "vehicle" else 1
        self.cam_asn = asn1tools.compile_files(
            "./cam_schema_carla.asn", "uper")

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
        encoded_data = self.cam_asn.encode("CAM", cam_data)
        return encoded_data

    def _data_deep_compare(self, new_data: Dict) -> bool:
        return all(self.data.get(key) == value for key, value in new_data.items())

    def decode_cam_message(self, encoded_data: bytes) -> Dict:
        decoded_data = self.cam_asn.decode("CAM", encoded_data)
        return decoded_data

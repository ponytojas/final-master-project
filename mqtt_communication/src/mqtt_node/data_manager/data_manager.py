import os
import logging

from mqtt_node.mqtt_client import MQTTClient


class DataManager:
    def __init__(self, mqtt_client: MQTTClient):
        self.mqtt_client = mqtt_client
        self.data = {}
        self.topic = os.getenv("MQTT_TOPIC") or "ros/data"

    def update_data(self, new_data: dict):
        if self._data_deep_compare(new_data):
            return
        self.data.update(new_data)
        self.mqtt_client.publish(self.topic, self.data)
        logging.debug("Published data to MQTT broker")

    def _data_deep_compare(self, new_data: dict) -> bool:
        if set(self.data.keys()) != set(new_data.keys()):
            return False
        for key in new_data:
            if key not in self.data or self.data[key] != new_data[key]:
                return False
        return True

class DataManager:
    def __init__(self, mqtt_client):
        self.mqtt_client = mqtt_client
        self.data = {}

    def update_data(self, new_data):
        self.data.update(new_data)
        self.mqtt_client.publish("ros/data", self.data)
        print("Datos actualizados y publicados: ", self.data)

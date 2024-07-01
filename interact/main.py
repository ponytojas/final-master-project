import time
import os
import logging
import asn1tools
import paho.mqtt.client as mqtt
from dotenv import load_dotenv
from node.InfrastructureNode import InfrastructureNode

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')

load_dotenv()

MESSAGE = {
    "header": {
        "protocolVersion": 1,
        "messageID": 2,  # Assuming 2 for DENM messages
        "stationID": 1
    },
    "denm": {
        "management": {
            "actionID": {
                "originatingStationID": 1,
                "sequenceNumber": 1
            },
            "detectionTime": int(time.time()),
            "referenceTime": int(time.time()),
            "termination": 'isCancellation'
        },
        "situation": {
            "informationQuality": 7,
            "eventType": 2
        },
        "location": {
            "traces": [
                {
                    "initialPosition": {
                        "latitude": 25,
                        "longitude": 26,
                        "positionConfidenceEllipse": {
                            "semiMajorConfidence": 0,
                            "semiMinorConfidence": 0,
                            "semiMajorOrientation": 0
                        }
                    },
                    "currencyPosition": {
                        "deltaLatitude": 0,
                        "deltaLongitude": 0,
                    },
                    "segmentCount": 1
                },
                {
                    "initialPosition": {
                        "latitude": 25,
                        "longitude": 57,
                        "positionConfidenceEllipse": {
                            "semiMajorConfidence": 0,
                            "semiMinorConfidence": 0,
                            "semiMajorOrientation": 0
                        }
                    },
                    "currencyPosition": {
                        "deltaLatitude": 0,
                        "deltaLongitude": 0,
                    },
                    "segmentCount": 1
                }
            ],
        }
    }
}


def on_connect(client, userdata, flags, rc):
    logging.debug(f"Connected with result code {rc}")
    if rc == 0:
        logging.info("Connection successful")
        client.connected_flag = True
    else:
        logging.error(f"Connection failed with code {rc}")
        client.connected_flag = False


def compile_asn_files(folder_path='asn_files'):
    schema_dict = {}
    for file_name in os.listdir(folder_path):
        if file_name.endswith('_schema.asn') or file_name.endswith('_carla.asn'):
            if file_name == 'cam_schema.asn' and 'cam_schema_carla.asn' in os.listdir(folder_path):
                continue
            key = file_name.split('_schema.asn')[0] if file_name.endswith(
                '_schema.asn') else file_name.split('_schema_carla.asn')[0]
            file_path = os.path.join(folder_path, file_name)
            schema_dict[key] = asn1tools.compile_files(file_path, "uper")
    return schema_dict


def send_denm_message(client: mqtt.Client, message: bytes):
    logging.info("Sending DENM message...")
    result = client.publish("v2x/infrastructure/denm", message)
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        logging.info("Message sent successfully")
    else:
        logging.error(f"Failed to send message, error code: {result.rc}")


def main():
    compilers = compile_asn_files()
    encoded_message = compilers['denm'].encode('DENM', MESSAGE)
    # write the encoded message to a file
    with open('encoded_denm_message.txt', 'wb') as f:
        f.write(encoded_message)


if __name__ == "__main__":
    main()

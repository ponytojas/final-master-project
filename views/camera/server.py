import carla
import random
import numpy as np
import cv2
import socket
import struct

colors = {}


def send_frame(client_socket, frame):
    # Compress the frame as JPEG
    _, buffer = cv2.imencode('.jpg', frame)
    # Convert to bytes
    frame_data = buffer.tobytes()
    # Send the frame size and data
    client_socket.sendall(struct.pack("L", len(frame_data)) + frame_data)


def main():
    # Connect to the Carla server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Get the map dimensions using spawn points
    spawn_points = world.get_map().get_spawn_points()
    min_x = min(spawn_points, key=lambda p: p.location.x).location.x
    max_x = max(spawn_points, key=lambda p: p.location.x).location.x
    min_y = min(spawn_points, key=lambda p: p.location.y).location.y
    max_y = max(spawn_points, key=lambda p: p.location.y).location.y

    # Calculate the center of the map
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    # Create a new RGB camera sensor
    camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_blueprint.set_attribute('image_size_x', '800')
    camera_blueprint.set_attribute('image_size_y', '600')
    camera_blueprint.set_attribute('fov', '90')

    # Set the sensor's transform to a top-down view
    sensor_transform = carla.Transform(
        carla.Location(x=center_x, y=center_y, z=175),
        carla.Rotation(pitch=-90)
    )

    # Spawn the sensor
    sensor = world.spawn_actor(camera_blueprint, sensor_transform)

    # Create a socket for streaming the video
    host = 'localhost'
    port = 9994
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Waiting for a client to connect on port {port}...")

    # Accept client connection
    client_socket, address = server_socket.accept()
    print(f"Connected to client: {address}")

    # Callback function to process the captured images
    def process_image(image):
        # Convert the raw image data to a numpy array
        frame = np.frombuffer(image.raw_data, dtype=np.uint8)
        frame = frame.reshape((image.height, image.width, 4))
        frame = frame[:, :, :3]  # Extract RGB channels

        # Create a writable copy of the frame
        frame_copy = frame.copy()

        # Get all vehicles in the world
        vehicles = world.get_actors().filter('vehicle.*')

        # Draw circles over vehicles
        for vehicle in vehicles:
            # Get the vehicle's location in world space
            vehicle_location = vehicle.get_location()

            # Get vehicle's ID
            vehicle_id = vehicle.id

            # Assign a random color if the vehicle is not yet in the dictionary
            if vehicle_id not in colors:
                colors[vehicle_id] = (random.randint(
                    0, 255), random.randint(0, 255), random.randint(0, 255))

            # Convert the vehicle's location to sensor space
            sensor_location = sensor.get_transform().location
            relative_location = vehicle_location - sensor_location
            pixel_x = int(relative_location.x * image.width / 150)
            pixel_y = int(-relative_location.y * image.height / 150)

            # Generate a random color for the circle
            color = colors[vehicle_id]

            # Draw the circle on the frame copy
            cv2.circle(frame_copy, (pixel_x + image.width // 2,
                       pixel_y + image.height // 2), 10, color, -1)

        # Send the frame copy over the network
        send_frame(client_socket, frame_copy)

    # Attach the callback function to the sensor
    sensor.listen(lambda image: process_image(image))

    # Keep the script running
    while True:
        pass


if __name__ == '__main__':
    main()

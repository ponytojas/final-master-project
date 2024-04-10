import carla
import random
import numpy as np
import cv2
from flask import Flask, Response

# Initialize CARLA client
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)  # seconds
world = client.get_world()

# Place an RGB camera sensor
blueprint_library = world.get_blueprint_library()
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '1200')
camera_bp.set_attribute('image_size_y', '800')
camera_bp.set_attribute('fov', '110')

# Adjust the location and rotation for the top-down view
# Note: You need to adjust the x, y, z, pitch, and yaw values according to your specific map and desired view.
camera_transform = carla.Transform(carla.Location(
    x=0, y=0, z=150), carla.Rotation(pitch=-90))
camera = world.spawn_actor(camera_bp, camera_transform)

# Initialize Flask application
app = Flask(__name__)

video_frame = None


actor_colors = {}


def draw_actors_on_image(image):
    global world, camera, actor_colors
    camera_transform = camera.get_transform()
    camera_location = camera_transform.location
    actors = [actor for actor in world.get_actors()
              if 'vehicle' in actor.type_id or 'walker' in actor.type_id or 'traffic_light' in actor.type_id]

    for actor in actors:
        actor_id = actor.id
        if actor_id not in actor_colors:
            actor_colors[actor_id] = (random.randint(
                0, 255), random.randint(0, 255), random.randint(0, 255))

        actor_location = actor.get_location()
        dx = camera_location.x + (actor_location.y * 0.95)
        dy = camera_location.y + (actor_location.x * 0.95)

        scale_factor = 3  # Adjust this scale factor as needed
        pixel_x = 600 + dx * scale_factor  # Adjust for image dimensions
        pixel_y = 400 - dy * scale_factor

        if 'vehicle' in actor.type_id:
            cv2.circle(image, (int(pixel_x), int(pixel_y)),
                       8, actor_colors[actor_id], -1)
        elif 'walker' in actor.type_id:
            points = np.array([[(int(pixel_x), int(pixel_y) - 10),
                                (int(pixel_x) - 5, int(pixel_y) + 5),
                                (int(pixel_x) + 5, int(pixel_y) + 5)]], np.int32)
            cv2.polylines(image, [points], isClosed=True,
                          color=actor_colors[actor_id], thickness=2)
        elif 'traffic_light' in actor.type_id:
            # Draw a square for traffic lights
            top_left = (int(pixel_x) - 10, int(pixel_y) - 10)
            bottom_right = (int(pixel_x) + 5, int(pixel_y) + 5)
            cv2.rectangle(image, top_left, bottom_right,
                          actor_colors[actor_id], -1)

    return image


def image_callback(image):
    global video_frame
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    video_frame = cv2.cvtColor(array, cv2.COLOR_BGR2BGRA)
    video_frame = draw_actors_on_image(
        video_frame)  # Draw vehicles on the image
    video_frame = cv2.resize(video_frame, (1200, 800))


camera.listen(image_callback)


def generate_video():
    global video_frame
    while True:
        if video_frame is not None:
            ret, encoded_image = cv2.imencode('.jpg', video_frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')


@app.route('/video')
def video():
    return Response(generate_video(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    finally:
        camera.stop()
        camera.destroy()

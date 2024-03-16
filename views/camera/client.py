import socket
import struct
import cv2
import numpy as np

# Server connection details
host = 'localhost'
port = 9994

# Create a socket and connect to the server
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))
print(f"Connected to server: {host}:{port}")

# Receive and display the video frames
while True:
    # Receive the frame size
    frame_size_data = client_socket.recv(struct.calcsize("L"))
    if not frame_size_data:
        break
    frame_size = struct.unpack("L", frame_size_data)[0]

    # Receive the frame data
    frame_data = b""
    while len(frame_data) < frame_size:
        remaining_size = frame_size - len(frame_data)
        frame_data += client_socket.recv(remaining_size)

    # Convert the frame data to a numpy array
    frame = np.frombuffer(frame_data, dtype=np.uint8)
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

    # Display the frame
    cv2.imshow("Carla Zenital View", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
client_socket.close()
cv2.destroyAllWindows()

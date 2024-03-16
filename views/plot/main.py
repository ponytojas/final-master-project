import carla
import matplotlib.pyplot as plt
plt.switch_backend('TkAgg')
import numpy as np

# Connect to the CARLA simulator
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# Get the map
carla_map = world.get_map()

# Plot the map
try:
    # Try converting map data to NumPy array
    map_data = np.array(carla_map.to_opendrive())
    map_image = map_data.reshape((map_data.shape[0], -1, 3))  # Reshape the map image based on the calculated size
except Exception as e:
    print(f"Error loading map image: {str(e)}")
    map_size = 1000
    map_image = np.zeros((map_size, map_size, 3))  # Create a blank image as a fallback

# Create a figure and axis for the plot
fig, ax = plt.subplots(figsize=(10, 10))  # Adjust the figure size as needed
ax.set_facecolor('white')  # Set the background color to white

# Find the map boundaries
waypoints = carla_map.generate_waypoints(2)
x_coords = [waypoint.transform.location.x for waypoint in waypoints]
y_coords = [waypoint.transform.location.y for waypoint in waypoints]
min_x, max_x = min(x_coords), max(x_coords)
min_y, max_y = min(y_coords), max(y_coords)

# Set the plot extent based on the map boundaries with a small margin
margin = 50  # Adjust the margin as needed
extent = [min_x - margin, max_x + margin, min_y - margin, max_y + margin]
ax.imshow(map_image, extent=extent, cmap='gray')

# Plot the roads
topology = carla_map.get_topology()
for segment in topology:
    x, y = [], []
    for i in range(len(segment) - 1):
        waypoint = segment[i]
        next_waypoint = segment[i + 1]
        
        # Check if the waypoints are connected by a straight road
        if waypoint.is_intersection or next_waypoint.is_intersection:
            # If either waypoint is an intersection, plot the segment
            location = waypoint.transform.location
            x.append(location.x)
            y.append(location.y)
        else:
            # If the waypoints are not intersections, interpolate points between them
            num_points = 10  # Adjust the number of interpolated points as needed
            x_vals = np.linspace(waypoint.transform.location.x, next_waypoint.transform.location.x, num_points)
            y_vals = np.linspace(waypoint.transform.location.y, next_waypoint.transform.location.y, num_points)
            x.extend(x_vals)
            y.extend(y_vals)
    
    ax.plot(x, y, 'r-', linewidth=1)

# Set the axis labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Zenith View of Vehicles and Roads')

# Add smaller grid squares
num_grid_lines = 20  # Adjust the number of grid lines as needed
ax.set_xticks(np.linspace(min_x, max_x, num_grid_lines))
ax.set_yticks(np.linspace(min_y, max_y, num_grid_lines))
ax.grid(True, linestyle='--', linewidth=0.5)

# Create a list to store the vehicle plot handles
vehicle_plots = []

# Define the update function
def update():
    # Clear the previous vehicle plots
    for plot in vehicle_plots:
        plot.remove()
    vehicle_plots.clear()

    # Get the updated vehicle positions
    vehicles = world.get_actors().filter('vehicle.*')

    # Plot the updated vehicle positions with green color
    for vehicle in vehicles:
        location = vehicle.get_location()
        plot = ax.plot(location.x, location.y, 'go', markersize=5)[0]
        vehicle_plots.append(plot)

    # Redraw the plot
    fig.canvas.draw()

    # Set a timer to call the update function again after a certain interval
    fig.canvas.flush_events()
    plt.pause(0.1)  # Adjust the interval as needed

# Start the plot update loop
while True:
    update()
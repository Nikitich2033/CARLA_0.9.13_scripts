import json
import matplotlib.pyplot as plt

# Load data from JSON file
with open('user_input/scenario_0.json', 'r') as f:
    data = json.load(f)

# Initialize lists to store data
timestamps = []
velocities = []
throttles = []

# Extract values from data
game_time = [entry['game_time'] for entry in data]
velocity = [entry['vehicle_velocity']['x'] for entry in data]
throttle = [entry['vehicle_throttle'] for entry in data]

# Plot velocity over time
plt.plot(game_time, velocity)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.show()

# Plot throttle over time
plt.plot(game_time, throttle)
plt.xlabel('Time (s)')
plt.ylabel('Throttle')
plt.show()
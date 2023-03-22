from scipy.spatial.distance import euclidean
from fastdtw import fastdtw
from dtw import *
import math
import json
import matplotlib.pyplot as plt
import numpy as np
import os

rated_objects = []
count = 0
for root, dirs, files in os.walk('user_input/NN_run/step2'):
    for file in files:
        if 'auto_scenario' in file:
        
            
            # Extract the number from the file name
            scenario_num = int(file.split('_')[-1].split('.')[0])
            print(scenario_num)
            # Open the corresponding route file
            with open(f'user_input/NN_run/step1/random_route_{scenario_num}.json', 'r') as f:
                trajectory = json.load(f)

            with open(f'user_input/NN_run/step2/auto_scenario_{scenario_num}.json') as f:
                driven_path_data = json.load(f)

          

            def get_trajectory_length(trajectory):
                # Extract the X and Y coordinates from each location
                x_coords = [location['X'] for location in trajectory]
                y_coords = [location['Y'] for location in trajectory]

                # Compute the Euclidean distances between consecutive points
                distances = [math.sqrt((x2 - x1)**2 + (y2 - y1)**2) for x1, y1, x2, y2 in zip(x_coords[:-1], y_coords[:-1], x_coords[1:], y_coords[1:])]

                # Return the total length of the trajectory
                return sum(distances)

            def path_length(path):
                length = 0
                for i in range(1, len(path)):
                    x1, y1 = path[i-1]
                    x2, y2 = path[i]
                    segment_length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
                    length += segment_length
                return length

    
            driven_path = [(d['vehicle_location']['x'], d['vehicle_location']['y']) for d in driven_path_data]
            
            # Plot the driven path
            plt.plot(*zip(*driven_path), label='Driven path')

            # Extract the X and Y coordinates from each location
            x_coords = [location['X'] for location in trajectory]
            y_coords = [location['Y'] for location in trajectory]

            # Plot the route on a graph
            plt.plot(x_coords, y_coords, label='Perfect trajectory')

            plt.title('Driven path vs Perfect trajectory')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.legend()
            plt.show()

            # Initialize lists to store data
            timestamps = []
            velocities = []
            throttles = []

            # Extract values from data
            game_time = [entry['game_time'] for entry in driven_path_data]
            velocity = [entry['vehicle_velocity']['x'] for entry in driven_path_data]
            throttle = [entry['vehicle_throttle'] for entry in driven_path_data]

            # Calculate average velocity
            avg_velocity = np.mean(velocity)
            print(f'Average velocity: {avg_velocity:.2f} m/s')

            # Calculate acceleration
            dt = np.mean(np.diff(game_time))
            acceleration = np.gradient(velocity, dt)
            avg_acceleration = np.mean(acceleration)
            print(f'Average acceleration: {avg_acceleration:.2f} m/s^2')
            max_acceleration = max(acceleration)
            print(f'Max acceleration: {max_acceleration:.2f} m/s^2')

            driven_path_length = path_length(driven_path)
            print(f"Driven path length: {driven_path_length:.2f} meters")
            
            ideal_trajectory_length = get_trajectory_length(trajectory)
            print(f"Ideal trajectory length: {ideal_trajectory_length:.2f} meters")


            last_object = driven_path_data[-1]
            num_collisions = last_object["unique_collisions"]
            solid_lane_crosses = last_object["solid_lane_crosses"]
            double_solid_lane_crosses = last_object["double_solid_lane_crosses"]

            def new_rate_driven_path(trajectory, ideal_trajectory_length, driven_path, driven_path_length,
                         avg_velocity, avg_acceleration, num_collisions, solid_lane_crosses, double_solid_lane_crosses):
    
                # Calculate the DTW distance between the trajectory and driven path
                distance, _ = fastdtw(trajectory, driven_path, dist=euclidean)

                # Set the maximum safety score
                #400 is the maximum length of route, at current Scenario Generator params 
                max_score = 1000 * (ideal_trajectory_length / 400) 

                # Calculate the safety score based on distance, velocity, and acceleration
                velocity_factor = 1  # Weighting factor for velocity
                acceleration_factor = 2  # Weighting factor for acceleration
                driven_length_factor = driven_path_length / ideal_trajectory_length  # Scaling factor based on completed length ratio
                ideal_trajectory_length_factor = ideal_trajectory_length / 10
                safety_score = max_score + ideal_trajectory_length_factor * driven_length_factor - distance - velocity_factor * avg_velocity - acceleration_factor * avg_acceleration

                # Penalize for number of collisions
                collision_penalty = 50
                safety_score -= num_collisions * collision_penalty

                # Penalize for solid lane crosses
                solid_lane_cross_penalty = 10
                safety_score -= solid_lane_crosses * solid_lane_cross_penalty

                # Penalize for double solid lane crosses
                double_solid_lane_cross_penalty = 30
                safety_score -= double_solid_lane_crosses * double_solid_lane_cross_penalty

                # Normalize safety_score to range 0-100
                safety_score_normalized = (safety_score / (max_score)) * 100
                safety_score_normalized = max(0, min(safety_score_normalized, 100))
                return safety_score_normalized



            # # # Plot velocity over time
            # plt.plot(game_time, velocity)
            # plt.xlabel('Time (s)')
            # plt.ylabel('Velocity (m/s)')
            # plt.show()

            # # Plot acceleration over time
            # plt.plot(game_time, acceleration)
            # plt.xlabel('Time (s)')
            # plt.ylabel('Acceleration (m/s^2)')
            # plt.show()

            perfect_trajectory_array = list(zip(x_coords, y_coords))
            driven_path_array = np.array(driven_path)

            # # calculated_score = rate_driven_path(perfect_trajectory_array,
                                                    
            #                                         driven_path_array,
                                                    
            #                                         avg_velocity,
            #                                         avg_acceleration,
            #                                         num_collisions,
            #                                        )
            
            calculated_score = new_rate_driven_path(perfect_trajectory_array,ideal_trajectory_length, driven_path_array, driven_path_length, 
                                     avg_velocity, avg_acceleration, num_collisions, solid_lane_crosses, double_solid_lane_crosses)
            
            print(f'Calculated score: {calculated_score:.2f}')
           
            # Open the file for reading
            with open('user_input/NN_run/step1/random_scenarios.json', 'r') as f:
                data = json.load(f)

            print(f'There are {len(data)} generated scenarios')
            
           
            # Find the object with the desired scenario_num
            for obj in data:
                if obj['scenario_num'] == scenario_num:
                    # Add the new variable
                    obj['calculated_score'] = calculated_score  # Replace 0 with your desired value
                    rated_objects.append(obj)

# Open the file for writing
with open('user_input/NN_run/step3/rated_scenarios.json', 'w') as f:
    json.dump(rated_objects, f,indent=4)
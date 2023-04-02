from scipy.spatial.distance import euclidean
from fastdtw import fastdtw
from dtw import *
import math
import json
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import os

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

import matplotlib.pyplot as plt

def generate_graph(perfect_trajectory_array, ideal_trajectory_length, driven_path_array, driven_path_length, game_time,
                   velocity, avg_acceleration, num_collisions, solid_lane_crosses, double_solid_lane_crosses,
                   acceleration, throttle, calculated_score, dtw_distance, scenario_num,scenario_data,):

    fig, axs = plt.subplots(2, 2, figsize=(10, 10))
    
    # Plot velocity over time
    axs[0, 0].plot(game_time, velocity)
    axs[0, 0].set(xlabel='Time (s)', ylabel='Velocity (m/s)')
    
    # Plot acceleration over time
    axs[0, 1].plot(game_time, acceleration)
    axs[0, 1].set(xlabel='Time (s)', ylabel='Acceleration (m/s^2)')
    
    # Plot throttle over time
    axs[1, 0].plot(game_time, throttle)
    axs[1, 0].set(xlabel='Time (s)', ylabel='Throttle')

    # Plot the driven path
    axs[1, 1].plot(*zip(*driven_path_array), label='Driven path')

    # Plot the route on a graph
    axs[1, 1].plot(*zip(*perfect_trajectory_array), label='Perfect trajectory')
    axs[1, 1].set(xlabel='X', ylabel='Y')
    axs[1, 1].legend()

    # Set the title with larger font and bold style
    fig.suptitle(f'Scenario {scenario_num}', fontsize=16, fontweight='bold')

    # Get additional variables from scenario data
   
    weather = scenario_data[scenario_num]["weather"]
    intersections = scenario_data[scenario_num]["intersections"]
    vehicle = scenario_data[scenario_num]["vehicle"]
    traffic = scenario_data[scenario_num]["traffic"]
    emergency = scenario_data[scenario_num]["emergency"]
    timeOfDay = scenario_data[scenario_num]["timeOfDay"]
    location = scenario_data[scenario_num]["location"]
    pedestrians = scenario_data[scenario_num]["pedestrians"]
    pedestrian_cross = scenario_data[scenario_num]["pedestrian_cross"]
    route_length = scenario_data[scenario_num]["route_length"]
    total_difficulty_rating = scenario_data[scenario_num]["total_difficulty_rating"]

    # Add the additional variables below the title
    additional_variables_text = f'Weather: {weather}, Intersections: {intersections}, Vehicle: {vehicle}, Traffic: {traffic}\n' \
                                 f'Emergency: {emergency}, Time of Day: {timeOfDay}, Location: {location}, Pedestrians: {pedestrians}\n' \
                                 f'Pedestrian Cross: {pedestrian_cross}, Route Length: {route_length:.2f}, Total Difficulty Rating: {total_difficulty_rating:.2f}'

    fig.text(0.5, 0.96, additional_variables_text, ha='center', va='top', fontsize=10)

    # Add text with all the new parameters at the bottom
    bottom_text = f'DTW Distance: {dtw_distance:.2f}  Calculated Score: {calculated_score:.2f}\n' \
              f'Ideal Trajectory Length: {ideal_trajectory_length:.2f}   Driven Path Length: {driven_path_length:.2f}\n' \
              f'Avg Acceleration: {avg_acceleration:.2f}\n' \
              f'Num Collisions: {num_collisions}\n' \
              f'Solid Lane Crosses: {solid_lane_crosses}\n' \
              f'Double Solid Lane Crosses: {double_solid_lane_crosses}\n'


    fig.text(0.5, 0.10, bottom_text, ha='center', va='top', fontsize=10)

    # Adjust spacing to avoid overlapping
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.15, hspace=0.4, wspace=0.4)

    # Save the figure as an image
    plt.savefig(f'step3/scenario_{scenario_num}_results.png', dpi=300)

    # Close the plot
    plt.close()

def generate_score_distribution_graph(rated_scenario_data):
    # Extract calculated scores
    calculated_scores = [scenario["calculated_score"] for scenario in rated_scenario_data]

    # Create a bell curve distribution graph
    sns.set(style="whitegrid")
    sns.histplot(calculated_scores, kde=True, bins=20)

    plt.title("Distribution of Calculated Scores")
    plt.xlabel("Calculated Score")
    plt.ylabel("Frequency")

    # Save the figure as an image
    plt.savefig(f'step3/calculated_safety_score_dist.png', dpi=300)

    # Close the plot
    plt.close()



def rate_driven_path(trajectory, ideal_trajectory_length, driven_path, driven_path_length,
                         avg_velocity, avg_acceleration, num_collisions, solid_lane_crosses, double_solid_lane_crosses):
    
                # Calculate the DTW distance between the trajectory and driven path
                distance, _ = fastdtw(trajectory, driven_path, dist=euclidean)

                # Set the maximum safety score
                max_score = 1000 * (ideal_trajectory_length / 400)

                # Calculate the safety score based on distance, velocity, and acceleration
                velocity_factor = 1  # Weighting factor for velocity
                acceleration_factor = 2  # Weighting factor for acceleration
                driven_length_factor = driven_path_length / ideal_trajectory_length  # Scaling factor based on completed length ratio
                ideal_trajectory_length_factor = ideal_trajectory_length / 10
                safety_score = max_score + ideal_trajectory_length_factor * driven_length_factor - distance - velocity_factor * avg_velocity - acceleration_factor * avg_acceleration

                # Penalize for number of collisions
                collision_penalty = 25
                safety_score -= num_collisions * collision_penalty

                # Penalize for solid lane crosses
                solid_lane_cross_penalty = 10
                safety_score -= solid_lane_crosses * solid_lane_cross_penalty

                # Penalize for double solid lane crosses
                double_solid_lane_cross_penalty = 20
                safety_score -= double_solid_lane_crosses * double_solid_lane_cross_penalty

                # Normalize safety_score to range 0-100
                
                safety_score_normalized = max(0, min(((safety_score / (max_score)) * 100), 100))
                return safety_score_normalized,distance


with open("step1/random_scenarios.json", "r") as file:
        scenario_data = json.load(file)

rated_objects = []
count = 0
for root, dirs, files in os.walk('step2'):
    for file in files:
        if 'auto_scenario' in file:
        
            # Extract the number from the file name
            scenario_num = int(file.split('_')[-1].split('.')[0])
            # print(scenario_num)
            # Open the corresponding route file
            with open(f'step1/random_route_{scenario_num}.json', 'r') as f:
                trajectory = json.load(f)

            with open(f'step2/auto_scenario_{scenario_num}.json') as f:
                driven_path_data = json.load(f)

            driven_path = [(d['vehicle_location']['x'], d['vehicle_location']['y']) for d in driven_path_data]
            
            # Extract the X and Y coordinates from each location
            x_coords = [location['X'] for location in trajectory]
            y_coords = [location['Y'] for location in trajectory]

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

            # Calculate acceleration
            dt = np.mean(np.diff(game_time))
            acceleration = np.gradient(velocity, dt)
            avg_acceleration = np.mean(acceleration)
           
            max_acceleration = max(acceleration)

            driven_path_length = path_length(driven_path)
            
            ideal_trajectory_length = get_trajectory_length(trajectory)

            last_object = driven_path_data[-1]
            num_collisions = last_object["unique_collisions"]
            solid_lane_crosses = last_object["solid_lane_crosses"]
            double_solid_lane_crosses = last_object["double_solid_lane_crosses"]

            perfect_trajectory_array = list(zip(x_coords, y_coords))
            driven_path_array = np.array(driven_path)
        
            calculated_score, DTW_distance = rate_driven_path(perfect_trajectory_array,
                                                    ideal_trajectory_length,
                                                    driven_path_array,
                                                    driven_path_length,
                                                    avg_velocity,
                                                    avg_acceleration,
                                                    num_collisions,
                                                    solid_lane_crosses,
                                                    double_solid_lane_crosses
                                                   )
            print(f'Calculated score: {calculated_score:.2f}')

            generate_graph(perfect_trajectory_array,ideal_trajectory_length, driven_path_array,driven_path_length, game_time, 
                            velocity, avg_acceleration, num_collisions, solid_lane_crosses,double_solid_lane_crosses,acceleration, 
                            throttle, calculated_score, DTW_distance, scenario_num,scenario_data)
           
            # Open the file for reading
            with open('step1/random_scenarios.json', 'r') as f:
                data = json.load(f)

            # Find the object with the desired scenario_num
            for obj in data:
                if obj['scenario_num'] == scenario_num:
                    # Add the new variables
                    obj['driven_path_length'] = driven_path_length
                    obj['unique_collisions'] = num_collisions
                    obj['solid_lane_crosses'] = solid_lane_crosses
                    obj['double_solid_lane_crosses'] = double_solid_lane_crosses
                    obj['calculated_score'] = calculated_score  # Replace 0 with your desired value
                    rated_objects.append(obj)

# Open the file for writing
with open('step3/rated_scenarios.json', 'w') as f:
    json.dump(rated_objects, f,indent=4)

with open("step3/rated_scenarios.json", "r") as file:
        rated_scenario_data = json.load(file)

generate_score_distribution_graph(rated_scenario_data)
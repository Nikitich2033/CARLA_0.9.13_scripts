import json
import numpy as np
import os
import re
import random
from sklearn.preprocessing import LabelEncoder, OneHotEncoder
from sklearn.neural_network import MLPRegressor
from sklearn.metrics import mean_squared_error
import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner


#these are the variables that should not be changed
weather = ['Sunny', 'Rain', 'Thunderstorm']
vehicle = ['Small', 'Truck', 'Van']
traffic = ['Heavy', 'Light', 'Medium']
emergency = ['Yes', 'No']
timeOfDay = ['Day', 'Night', 'Dawn', 'Dusk']
location = ['Urban', 'Country', 'Downtown']
intersections = [0, 1, 2, 3, 4]
pedestrians = [True, False]
pedestrian_cross = [True, False]
route_length = [100, 150, 200, 250, 300, 350, 400]


class EnhancedScenarioGenerator():
    def __init__(self, weather, vehicle, traffic, emergency, timeOfDay, location, intersections, pedestrians, pedestrian_cross, route_length, num_scenarios):
        self.weather = weather
        self.vehicle = vehicle
        self.traffic = traffic
        self.emergency = emergency
        self.timeOfDay = timeOfDay
        self.location = location
        self.intersections = intersections
        self.pedestrians = pedestrians
        self.pedestrian_cross = pedestrian_cross
        self.num_scenarios = num_scenarios
        self.route_length = route_length
        
    def train_regressor_on_rated(self,data):

        # Extract features and target variable
        X = []
        y = []
        for obj in data:
            X.append([
                obj['weather'],
                obj['vehicle'],
                obj['traffic'],
                obj['emergency'],
                obj['timeOfDay'],
                obj['location'],
                obj['intersections'],
                int(obj['pedestrians']),
                int(obj['pedestrian_cross']),
                obj['rounded_route_length'],
                
            ])
            
            y.append(obj['calculated_score'])

        X = np.array(X)
        y = np.array(y)

        # Encode categorical features
        categorical_features = [0, 1, 2, 3, 4, 5]
        self.encoders = []
        for i in categorical_features:
            le = LabelEncoder()
            ohe = OneHotEncoder(sparse=False)
            X[:, i] = le.fit_transform(X[:, i])
            self.encoders.append((le, ohe))
        X = ohe.fit_transform(X)

        # Split data into training and test sets
        train_size = int(0.9 * len(X))
        X_train, X_test = X[:train_size], X[train_size:]
        y_train, y_test = y[:train_size], y[train_size:]

        # Train neural network regressor
        self.nn = MLPRegressor(hidden_layer_sizes=(1000,), max_iter=10000000)
        self.nn.fit(X_train, y_train)

        # Make predictions on test set
        y_pred = self.nn.predict(X_test)

        # Evaluate model performance (e.g. using mean squared error)
       
        mse = mean_squared_error(y_test, y_pred)
        rmse = np.sqrt(mse)
        print(f'MSE: {mse:.2f}')
        print(f'RMSE: {rmse:.2f}')

    def generate_scenarios_with_threshold(self,data,difficulty_threshold):

        # Create dictionaries that map each variable value to a difficulty rating
        weather_difficulty = {'Sunny': 1, 'Rain': 5, 'Thunderstorm': 8}
        vehicle_difficulty = {'Small': 1, 'Truck': 2, 'Van': 3}
        traffic_difficulty = {'Heavy': 6, 'Light': 1, 'Medium': 3}
        emergency_difficulty = {'Yes': 10, 'No': 0}
        timeOfDay_difficulty = {'Day': 1, 'Night': 5, 'Dawn': 2, 'Dusk': 2}
        location_difficulty = {'Urban': 4, 'Country': 1, 'Downtown': 3}
        pedestrians_difficulty = {True: 0.5, False: 0}
        pedestrian_cross_difficulty = {True: 10, False: 0}

        filtered_scenarios = []
        directory = 'step4'
        
        highest_number = 0

        # Get the highest numbered route file
        for filename in os.listdir(directory):
            match = re.search(r'filtered_route_(\d+)', filename)
            if match:
                number = int(match.group(1))
                if number > highest_number:
                    highest_number = number

        if highest_number > 0:
            highest_number += 1

        counter = highest_number

        # Generate scenarios
        while counter < self.num_scenarios:
            print(f'Generating scenario {counter}...')
            # Load existing data from file
            with open('step4/filtered_scenarios.json', 'r') as f:
                filtered_scenarios = json.load(f)

            
            # Randomly choose values for each variable
            weather_choice = random.choice(self.weather)
            vehicle_choice = random.choice(self.vehicle)
            traffic_choice = random.choice(self.traffic)
            emergency_choice = random.choice(self.emergency)
            timeOfDay_choice = random.choice(self.timeOfDay)
            location_choice = random.choice(self.location)
            intersections_choice = random.choice(self.intersections)
            pedestrians_choice = random.choice(self.pedestrians)
            pedestrian_cross_choice = random.choice(self.pedestrian_cross)
            route_length_choice = random.choice(self.route_length)

            # Connect to Carla server
            client = carla.Client("localhost", 2000)

            # Load the world based on location choice
            if location_choice == "Downtown":
                world = client.load_world('Town05')
            if location_choice == "Urban":
                world = client.load_world('Town03')
            if location_choice == "Country":
                world = client.load_world('Town07')
            
            map = world.get_map()
            grp = GlobalRoutePlanner(map, 2)

            waypoints = map.generate_waypoints(2.0)

            # Function to calculate route length
            def get_route_length(route):
                length = 0
                for i in range(len(route) - 1):
                    length += route[i][0].transform.location.distance(route[i + 1][0].transform.location)
                return length

            found = False

            # Generate routes with the desired parameters
            for start_waypoint in waypoints:
                for end_waypoint in waypoints:

                    route = grp.trace_route(start_waypoint.transform.location, end_waypoint.transform.location)
                    found_route_length = get_route_length(route)

                    # Get unique road IDs
                    road_ids = list(set(waypoint[0].road_id for waypoint in route))
                    junction_road_ids = []
                    junctions_on_route = 0

                    # Count the number of junctions on the route
                    if intersections_choice > 0:
                        for road_id in road_ids:
                            for waypoint in route:
                                if waypoint[0].road_id == road_id and waypoint[0].is_junction:
                                    junction_road_ids.append(road_id)

                        junctions_on_route = len(list(set(junction_road_ids)))

                    # Check if the route meets the length and junction requirements
                    if (found_route_length > (route_length_choice - (route_length_choice * 0.2))
                            and found_route_length < (route_length_choice + (route_length_choice * 0.2))
                            and junctions_on_route >= intersections_choice):
                        print(f'Found route with length {found_route_length} and {junctions_on_route} junctions')

                        found = True
                        start_waypoint = start_waypoint.transform.location
                        end_waypoint = end_waypoint.transform.location

                        junctions_on_route = len(list(set(junction_road_ids)))

                        # Calculate the total difficulty rating
                        total_difficulty_rating = (weather_difficulty[weather_choice] +
                                                vehicle_difficulty[vehicle_choice] +
                                                traffic_difficulty[traffic_choice] +
                                                emergency_difficulty[emergency_choice] +
                                                timeOfDay_difficulty[timeOfDay_choice] +
                                                location_difficulty[location_choice] +
                                                (junctions_on_route * 5) +
                                                pedestrians_difficulty[pedestrians_choice] +
                                                pedestrian_cross_difficulty[pedestrian_cross_choice] +
                                                found_route_length)

                        # Create a scenario dictionary with the chosen variable values and total difficulty rating
                        scenario = {
                            'scenario_num': counter,
                            'weather': weather_choice,
                            'vehicle': vehicle_choice,
                            'traffic': traffic_choice,
                            'emergency': emergency_choice,
                            'timeOfDay': timeOfDay_choice,
                            'location': location_choice,
                            "intersections": junctions_on_route,
                            "pedestrians": pedestrians_choice,
                            "pedestrian_cross": pedestrian_cross_choice,
                            "start_x": start_waypoint.x,
                            "start_y": start_waypoint.y,
                            "end_x": end_waypoint.x,
                            "end_y": end_waypoint.y,
                            "rounded_route_length": route_length_choice,
                            "route_length": found_route_length,
                            "total_difficulty_rating": total_difficulty_rating
                        }
                        
                        # Extract features
                        X_unrated = []
                        
                        X_unrated.append([
                            scenario['weather'],
                            scenario['vehicle'],
                            scenario['traffic'],
                            scenario['emergency'],
                            scenario['timeOfDay'],
                            scenario['location'],
                            scenario['intersections'],
                            int( scenario['pedestrians']),
                            int( scenario['pedestrian_cross']),
                            scenario['rounded_route_length']
                        ])

                        X_unrated = np.array(X_unrated)

                        # Encode categorical features
                        categorical_features = [0, 1, 2, 3, 4, 5]
                            
                        # Encode categorical features
                        for i in categorical_features:
                            le, ohe = self.encoders[i]
                            X_unrated[:, i] = le.transform(X_unrated[:, i])
                        X_unrated = ohe.transform(X_unrated)

                        # Make predictions on unrated scenarios
                        y_pred_unrated = self.nn.predict(X_unrated)

                        if  y_pred_unrated[0] < difficulty_threshold:
                            print(f"W: {scenario['weather']}, V: {scenario['vehicle']}, T: {scenario['traffic']}, E: {scenario['emergency']}, TD: {scenario['timeOfDay']}, L: {scenario['location']}, I: {scenario['intersections']}, P: {int(scenario['pedestrians'])}, PC: {int(scenario['pedestrian_cross'])}, RL: {scenario['route_length']}")
                            print(f"Predicted Score: {y_pred_unrated[0]:.2f}\n")
                            
                            
                            scenario['predicted_score'] = y_pred_unrated[0]
                            filtered_scenarios.append(scenario)

                                # Write updated data to JSON file
                            with open('step4/filtered_scenarios.json', 'w') as f:
                                json.dump(filtered_scenarios, f, indent=4) 


                            # Create a list of dictionaries representing each location in the route
                            locations = []
                            for waypoint, _ in route:
                                location = waypoint.transform.location
                                locations.append({'X': location.x, 'Y': location.y})

                            # Save the locations to a JSON file
                            with open(f'step4/filtered_route_{counter}.json', 'w') as f:
                                json.dump(locations, f,indent=4)
                            
                            counter += 1
                                 
                        break
                if found:
                    break


# Load data from JSON file with data that the regressor need to be trained on
with open('step3/rated_scenarios.json', 'r') as f:
    rated_data = json.load(f)

 # Check if file exists, create it if it doesn't
if not os.path.exists('step4/filtered_scenarios.json'):
    with open('step4/filtered_scenarios.json', 'w') as f:
        json.dump([], f)

# Load existing data from JSON file where the scenarios will be saved
with open('step4/filtered_scenarios.json', 'r') as f:
    filtered_data = json.load(f)

# Create a scenario generator with specified number of scenarios to generate
num_scenarios = 20

scenario_generator = EnhancedScenarioGenerator(weather, vehicle, traffic, emergency, timeOfDay, location, intersections, pedestrians, pedestrian_cross, route_length,num_scenarios)
scenario_generator.train_regressor_on_rated(rated_data)
scenario_generator.generate_scenarios_with_threshold(filtered_data,25)
 
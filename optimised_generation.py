import json
import random
from sklearn.multioutput import MultiOutputRegressor
from sklearn.ensemble import RandomForestRegressor

class ScenarioGenerator:
    def __init__(self, weather, road, vehicle, traffic, emergency, timeOfDay, location, num_cars, intersection, pedestrians, pedestrian_cross, num_scenarios):
        self.weather = weather
        self.road = road
        self.vehicle = vehicle
        self.traffic = traffic
        self.emergency = emergency
        self.timeOfDay = timeOfDay
        self.location = location
        self.num_cars = num_cars
        self.intersection = intersection
        self.pedestrians = pedestrians
        self.pedestrian_cross = pedestrian_cross
        self.num_scenarios = num_scenarios

    def generate_scenarios(self):
        generated_scenarios = []

        for i in range(self.num_scenarios):
            scenario = {
                'weather': random.choice(self.weather),
                'road': random.choice(self.road),
                'vehicle': random.choice(self.vehicle),
                'traffic': random.choice(self.traffic),
                'emergency': random.choice(self.emergency),
                'timeOfDay': random.choice(self.timeOfDay),
                'location': random.choice(self.location),
                "num_cars": random.choice(self.num_cars),
                "intersection": random.choice(self.intersection),
                "pedestrians": random.choice(self.pedestrians),
                "pedestrian_cross": random.choice(self.pedestrian_cross),
            }
            generated_scenarios.append(scenario)

        with open('user_input/scenarios.json', 'w') as f:
            json.dump(generated_scenarios, f, indent=4)
        
        return generated_scenarios


# Define the lists of possible values for each input variable
weather = ['Sunny', 'Rain', 'Thunderstorm']
road = ['Highway', 'City', 'Country']
vehicle = ['Car', 'Truck']
traffic = ['Heavy', 'Light']
emergency = ['Yes', 'No']
timeOfDay = ['Day', 'Night']
location = ['Urban', 'Rural']
num_cars = [5,10,15,2,1]
intersection = [True, False]
pedestrians = [True,False]
pedestrian_cross = [True, False]

# Define the number of scenarios to generate
num_scenarios = 100

# Generate the scenarios using the ScenarioGenerator class
scenario_generator = ScenarioGenerator(weather, road, vehicle, traffic, emergency, timeOfDay, location, num_cars, intersection, pedestrians, pedestrian_cross, num_scenarios)
scenarios = scenario_generator.generate_scenarios()

# Convert the scenarios into feature vectors (X) and target values (Y)
X = []
Y = []
for scenario in scenarios:
    x = [
        weather.index(scenario['weather']),
        road.index(scenario['road']),
        vehicle.index(scenario['vehicle']),
        traffic.index(scenario['traffic']),
        emergency.index(scenario['emergency']),
        timeOfDay.index(scenario['timeOfDay']),
        location.index(scenario['location']),
        num_cars.index(scenario['num_cars']),
        intersection.index(scenario['intersection']),
        pedestrians.index(scenario['pedestrians']),
        pedestrian_cross.index(scenario['pedestrian_cross'])
    ]
    X.append(x)
    Y.append([
        scenario['num_cars'],
        scenario['intersection'],
        scenario['pedestrians'],
        scenario['pedestrian_cross']
    ])

# Train a MultiOutputRegressor using a RandomForestRegressor estimator
regressor = MultiOutputRegressor(RandomForestRegressor(n_estimators=100))
regressor.fit(X, Y)
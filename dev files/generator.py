import json
import random

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
                "num_cars": random.choice(num_cars),
                "intersection": random.choice(self.intersection),
                "pedestrians": random.choice(self.pedestrians),
                "pedestrian_cross": random.choice(self.pedestrian_cross),
            }
            generated_scenarios.append(scenario)

        with open('user_input/scenarios.json', 'w') as f:
            json.dump(generated_scenarios, f, indent=4)
        
        return generated_scenarios

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
num_scenarios = 100


scenario_generator = ScenarioGenerator(weather, road, vehicle, traffic, emergency, timeOfDay, location, num_cars, intersection, pedestrians, pedestrian_cross, num_scenarios)
scenarios = scenario_generator.generate_scenarios()


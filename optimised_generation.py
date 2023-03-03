import json
import random

class ScenarioGenerator:
    def __init__(self, weather, vehicle, traffic, emergency, timeOfDay, location, intersections, pedestrians, pedestrian_cross,route_length, num_scenarios):
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

    def generate_scenarios(self):
        generated_scenarios = []

        for i in range(self.num_scenarios):
            scenario = {
                'weather': random.choice(self.weather),
                'vehicle': random.choice(self.vehicle),
                'traffic': random.choice(self.traffic),
                'emergency': random.choice(self.emergency),
                'timeOfDay': random.choice(self.timeOfDay),
                'location': random.choice(self.location),
                "intersections": random.choice(self.intersections),
                "pedestrians": random.choice(self.pedestrians),
                "pedestrian_cross": random.choice(self.pedestrian_cross),
                "route_length": random.choice(self.route_length),

            }
            generated_scenarios.append(scenario)

        with open('user_input/scenarios.json', 'w') as f:
            json.dump(generated_scenarios, f, indent=4)
        
        return generated_scenarios

weather = ['Sunny', 'Rain', 'Thunderstorm']
vehicle = ['Small', 'Truck','Van']
traffic = ['Heavy', 'Light','Medium']
emergency = ['Yes', 'No']
timeOfDay = ['Day', 'Night','Dawn','Dusk']
location = ['Urban', 'Country','Downtown']
intersections = [0,1,2,3,4,5]
pedestrians = [True,False]
pedestrian_cross = [True, False]
route_length = [50,100,120,200,250,300,400,500,600]
num_scenarios = 100


scenario_generator = ScenarioGenerator(weather, vehicle, traffic, emergency, timeOfDay, location, intersections, pedestrians, pedestrian_cross,route_length, num_scenarios)
scenarios = scenario_generator.generate_scenarios()


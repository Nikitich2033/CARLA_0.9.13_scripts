import json
import random
import pymoo

class ScenarioGenerator:
    def __init__(self, weather, road, vehicle, traffic, emergency, time, location, num_cars, intersection, pedestrian_cross, num_scenarios):
        self.weather = weather
        self.road = road
        self.vehicle = vehicle
        self.traffic = traffic
        self.emergency = emergency
        self.time = time
        self.location = location
        self.num_cars = num_cars
        self.intersection = intersection
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
                'time': random.choice(self.time),
                'location': random.choice(self.location),
                "intersection": random.choice(self.intersection),
                "pedestrian_cross": random.choice(self.pedestrian_cross),
                "num_cars": random.choice(num_cars)
            }
            generated_scenarios.append(scenario)

        with open('user_input/scenarios.json', 'w') as f:
            json.dump(generated_scenarios, f, indent=4)
        
        return generated_scenarios

weather = ['Sunny', 'Rainy', 'Snowy']
road = ['Highway', 'City']
vehicle = ['Car', 'Truck']
traffic = ['Heavy', 'Light']
emergency = ['Yes', 'No']
time = ['Day', 'Night']
location = ['Urban', 'Rural']
num_cars = [5,10,15,0,2,1]
intersection = [True, False]
pedestrian_cross = [True, False]
num_scenarios = 100


scenario_generator = ScenarioGenerator(weather, road, vehicle, traffic, emergency, time, location, num_cars, intersection, pedestrian_cross, num_scenarios)
scenarios = scenario_generator.generate_scenarios()

from pymoo.core.problem import Problem 
from pymoo.core.sampling import Sampling

class ScenarioEvaluator(Problem):

    def init(self, scenarios):
        super().init(n_var=7, n_obj=1, n_constr=0, xl=0, xu=1, type_var=pymoo.types.CATEGORICAL)
        self.scenarios = scenarios

    def _evaluate(self, x, out, *args, **kwargs):
        scenario = self.scenarios[int(x[0])]
        weather_coverage = x[1]
        road_coverage = x[2]
        vehicle_coverage = x[3]
        traffic_coverage = x[4]
        emergency_coverage = x[5]
        time_coverage = x[6]
        location_coverage = x[7]

        # Calculate diversity and coverage score for each variable
        weather_score = 1 if scenario['weather'] == 'Sunny' else 0
        road_score = 1 if scenario['road'] == 'Highway' else 0
        vehicle_score = 1 if scenario['vehicle'] == 'Car' else 0
        traffic_score = 1 if scenario['traffic'] == 'Heavy' else 0
        emergency_score = 1 if scenario['emergency'] == 'Yes' else 0
        time_score = 1 if scenario['time'] == 'Day' else 0
        location_score = 1 if scenario['location'] == 'Urban' else 0

         # Calculate overall diversity and coverage score
        diversity_score = weather_coverage * weather_score + road_coverage * road_score + \
                      vehicle_coverage * vehicle_score + traffic_coverage * traffic_score + \
                      emergency_coverage * emergency_score + time_coverage * time_score + \
                      location_coverage * location_score
        
        print("Diversity score: "+  diversity_score)

        out["F"] = diversity_score
        

scenario_evaluator = ScenarioEvaluator(len(scenarios))
# sampling = Sampling.do("random")
from pymoo.util import plotting
from pymoo.operators.sampling.rnd import FloatRandomSampling

# initial_solutions = Sampling.do(scenario_evaluator, 100,100)

problem = scenario_evaluator

sampling = FloatRandomSampling()

X = sampling(scenario_evaluator, 200).get("X")
plotting.plot(X, no_fill=True)

from pymoo.algorithms.moo.nsga2 import NSGA2

algorithm = NSGA2(pop_size=100)
result = algorithm.solve(scenario_evaluator, termination=("n_gen", 200))

# print(result)
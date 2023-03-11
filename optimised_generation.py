import json
import random
from agents.navigation.global_route_planner import GlobalRoutePlanner
import carla
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
        

         # Create dictionaries that map each variable value to a difficulty rating
        weather_difficulty = {'Sunny': 1, 'Rain': 5, 'Thunderstorm': 8}
        vehicle_difficulty = {'Small': 1, 'Truck': 2,'Van': 3}
        traffic_difficulty = {'Heavy': 6, 'Light': 1,'Medium': 3}
        emergency_difficulty = {'Yes': 10, 'No': 0}
        timeOfDay_difficulty = {'Day': 1, 'Night':5,'Dawn':2,'Dusk':2}
        location_difficulty = {'Urban':4 , 'Country' :1 ,'Downtown' :3 }
        intersections_difficulty = {0:1 ,1:2 ,2:3 ,3:4 ,4:5 ,5:6 }
        pedestrians_difficulty = {True: 0.5 ,False : 0}
        pedestrian_cross_difficulty = {True : 10, False : 0 }
        route_length_difficulty ={50 :0.5 ,100 :1.5,200 : 2 ,250:2.5,300:3,400:4,500:5,600:6}

        generated_scenarios = []
        for i in range(self.num_scenarios):
            # Randomly choose values for each variable
            weather_choice = random.choice(self.weather)
            vehicle_choice = random.choice(self.vehicle)
            traffic_choice = random.choice(self.traffic)
            emergency_choice= random.choice(self.emergency)
            timeOfDay_choice= random.choice(self.timeOfDay)
            location_choice= random.choice(self.location)
            intersections_choice= random.choice(self.intersections)
            pedestrians_choice= random.choice(self.pedestrians)
            pedestrian_cross_choice= random.choice(self.pedestrian_cross)
            route_length_choice=random.choice(self.route_length)

            
            client = carla.Client("localhost", 2000)
            
            if location_choice == "Downtown":
                world = client.load_world('Town05')
            if location_choice == "Urban":
                world = client.load_world('Town03')
            if location_choice == "Country":
                world = client.load_world('Town07')
            
            map = world.get_map()
            grp = GlobalRoutePlanner(map,2)

          
            waypoints = map.generate_waypoints(2.0)


            def get_route_length(route):
                length = 0
                for i in range(len(route) - 1):
                    length += route[i][0].transform.location.distance(route[i + 1][0].transform.location)
                return length
            
            start_waypoint = 0
            end_waypoint = 0
            for start_waypoint in waypoints:
                for end_waypoint in waypoints:
                    # print(start_waypoint.transform.location.distance(end_waypoint.transform.location))
                    route = grp.trace_route(start_waypoint.transform.location, end_waypoint.transform.location)
                    found_route_length = get_route_length(route)
                
                    
                    # Get unique road IDs
                    road_ids = list(set(waypoint[0].road_id for waypoint in route))
                    junction_road_ids = []
                    junctions_on_route = 0
                    if intersections_choice > 0:
                        for road_id in road_ids:
                            # waypoints = map.generate_waypoints(2.0)
                            for waypoint in route:
                                if waypoint[0].road_id == road_id and waypoint[0].is_junction:
                                    junction_road_ids.append(road_id)
                                    # draw_waypoints(waypoints, road_id=road_id, life_time=300)
                        junctions_on_route = len(list(set(junction_road_ids)))

                    if (found_route_length > (route_length_choice-(route_length_choice*0.2)) 
                        and found_route_length < (route_length_choice+(route_length_choice*0.2))
                        and junctions_on_route >= intersections_choice):
                        # print("wassup")
                        found = True
                        start_waypoint = start_waypoint.transform.location
                        end_waypoint = end_waypoint.transform.location

                        # Get unique road IDs
                        road_ids = list(set(waypoint[0].road_id for waypoint in route))

                        junction_road_ids = []
                        junctions_on_route = 0
                        for road_id in road_ids:
                            # waypoints = map.generate_waypoints(2.0)
                            for waypoint in route:
                                if waypoint[0].road_id == road_id and waypoint[0].is_junction:
                                    junction_road_ids.append(road_id)
                    
                        junctions_on_route = len(list(set(junction_road_ids)))
                        
                        # print(f"Need intersections: {intersections_choice}")
                        # print(f"Result intersections: {junctions_on_route}")
                        # # print(f"Unique road IDs along the route: {road_ids}")
                        # print(f"Euclidean Distance between: {start_waypoint.distance(end_waypoint)}")
                        # print(f"Need to find length: {route_length_choice}")
                        # print(f"Found Route length: {found_route_length}")


                        total_difficulty_rating = (weather_difficulty[weather_choice] +
                            vehicle_difficulty[vehicle_choice] +
                            traffic_difficulty[traffic_choice] +
                            emergency_difficulty[emergency_choice] +
                            timeOfDay_difficulty[timeOfDay_choice] +
                            location_difficulty[location_choice]+
                            (junctions_on_route*5)+
                            pedestrians_difficulty[pedestrians_choice]+
                            pedestrian_cross_difficulty[pedestrian_cross_choice]+
                                    found_route_length)
                        
                        # Create a scenario dictionary with the chosen variable values and total difficulty rating
                        scenario = {
                            'scenario_num': i,
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
                            "route_length": found_route_length,
                            "total_difficulty_rating": total_difficulty_rating
                        }

                        print(scenario)

                        # Create a list of dictionaries representing each location in the route
                        locations = []
                        for waypoint, _ in route:
                            location = waypoint.transform.location
                            locations.append({'X': location.x, 'Y': location.y})

                        # Save the locations to a JSON file
                        with open(f'user_input/route_{i}.json', 'w') as f:
                            json.dump(locations, f,indent=4)

                        generated_scenarios.append(scenario)
                        break
                        # build path using the route
                if found:
                    break

        # print(len(generated_scenarios))
        with open('user_input/scenarios.json', 'w') as f:
            json.dump(generated_scenarios, f, indent=4)
        
        return generated_scenarios

weather = ['Sunny', 'Rain', 'Thunderstorm']
vehicle = ['Small', 'Truck','Van']
traffic = ['Heavy', 'Light','Medium']
emergency = ['Yes', 'No']
timeOfDay = ['Day', 'Night','Dawn','Dusk']
location = ['Urban', 'Country','Downtown']
intersections = [0,1,2,3,4]
pedestrians = [True,False]
pedestrian_cross = [True, False]
route_length = [100,150,200,250,300,350,400]
num_scenarios = 3


scenario_generator = ScenarioGenerator(weather, vehicle, traffic, emergency, timeOfDay, location, intersections, pedestrians, pedestrian_cross,route_length, num_scenarios)
scenarios = scenario_generator.generate_scenarios()


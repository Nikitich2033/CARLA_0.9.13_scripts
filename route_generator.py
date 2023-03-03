import carla
import json
from agents.navigation.global_route_planner import GlobalRoutePlanner
import random
import math
from agents.navigation.basic_agent import BasicAgent
from carla import WeatherParameters

 # Open the JSON file
with open("user_input/scenarios.json", "r") as file:
    scenario_data = json.load(file)

# Initialize variables with the values from the JSON file

for scenario_num in range(len(scenario_data)):
    try:
        print("new scenario: "+ str(scenario_num))
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

        # Print the values of the variables

        print("Weather: ", weather)
        print("Intersections: ", intersections)
        print("Vehicle: ", vehicle)
        print("Traffic: ", traffic)
        print("Emergency: ", emergency)
        print("Time: ", timeOfDay)
        print("Location: ", location)
        print("Pedestrians: ", pedestrians)
        print("Pedestrian cross: ", pedestrian_cross)
        print("Route length: ", route_length)

        client = carla.Client("localhost", 2000)

        if location == "Downtown":
            world = client.load_world('Town05')
        if location == "Urban":
            world = client.load_world('Town03')
        if location == "Country":
            world = client.load_world('Town07')


        map = world.get_map()


        cloudiness=0.0,
        precipitation=0.0,
        sun_altitude_angle=70.0,   # 70 degrees is around noon

        # Set the weather conditions
        if weather == "Sunny":
            cloudiness=10
            precipitation=0.0
            precipitation_deposits=0

        elif weather == "Rain":
            cloudiness=80
            precipitation=60.0
            precipitation_deposits=30
            
        elif weather == "Thunderstorm":
            cloudiness=100
            precipitation=90.0
            precipitation_deposits=70
            

        if timeOfDay == "Day":
            sun_altitude_angle=70.0
        elif timeOfDay == "Night":
            sun_altitude_angle=-30.0
        elif timeOfDay == "Dawn":
            sun_altitude_angle=20.0
        elif timeOfDay == "Dusk":
            sun_altitude_angle=110.0




        weather_params = WeatherParameters(
            cloudiness=cloudiness,
            precipitation=precipitation,
            sun_altitude_angle=sun_altitude_angle,  
            precipitation_deposits=precipitation_deposits 
        )

        # Set the weather in the simulation
        world.set_weather(weather_params)


        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        traffic_manager.set_synchronous_mode(True)

        grp = GlobalRoutePlanner(map,2)

        spawn_points = map.get_spawn_points()

        # Set the desired number of junctions
        num_junctions = intersections

        # Keep generating routes until we find one that goes through the desired number of junctions
        while True:
            start_pose = random.choice(spawn_points)
            end_pose = random.choice(spawn_points)

            waypoints = grp.trace_route(start_pose.location, end_pose.location)

            # Count the number of junctions along the route
            junction_count = 0
            for waypoint in waypoints:
                if waypoint[0].is_junction:
                    junction_count += 1

            # Check if we found a route that goes through the desired number of junctions
            if junction_count == num_junctions:
                break



        waypoints = map.generate_waypoints(2.0)

        def draw_waypoints(waypoints, road_id=None, life_time=50.0):
            spawned = False 
            for waypoint in waypoints:
                
                if(waypoint.road_id == road_id):
                    world.debug.draw_string(waypoint.transform.location, "JUNC", draw_shadow=False,
                                            color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                            persistent_lines=True)

        def get_route_length(route):
            length = 0
            for i in range(len(route) - 1):
                length += route[i][0].transform.location.distance(route[i + 1][0].transform.location)
            return length
        
        print(f"Need to find: {route_length}")
        print(f"Need intersections: {intersections}")
        for start_waypoint in waypoints:
            for end_waypoint in waypoints:
                # print(start_waypoint.transform.location.distance(end_waypoint.transform.location))
                route = grp.trace_route(start_waypoint.transform.location, end_waypoint.transform.location)
                found_route_length = get_route_length(route)
              
                # junction_count = 0
                # for waypoint in route:
                #     if waypoint[0].is_junction:
                #         junction_count += 1

                if found_route_length > (route_length-(route_length*0.2)) and found_route_length < (route_length+(route_length*0.2)):
                    found = True
                    # Get unique road IDs
                    road_ids = list(set(waypoint[0].road_id for waypoint in route))
                    junction_road_ids = []
                    for road_id in road_ids:
                        # waypoints = map.generate_waypoints(2.0)
                        for waypoint in route:
                            if waypoint[0].road_id == road_id and waypoint[0].is_junction:
                                junction_road_ids.append(road_id)
                                draw_waypoints(waypoints, road_id=road_id, life_time=300)

                    junctions_on_route = len(list(set(junction_road_ids)))
                    print(f"Num of junctions: {junctions_on_route}")
                    print(f"Unique road IDs along the route: {road_ids}")
                    print(f"Distance between: {start_waypoint.transform.location.distance(end_waypoint.transform.location)}")
                    print(f"Route length: {found_route_length}")
                    break
                    # build path using the route
            if found:
                break



        # The 'waypoints' variable now contains a list of waypoints that define a route between 'start_pose' and 'end_pose' that goes through 'num_junctions' junctions.

        i = 0
        for w in route:
            if i % 10 == 0:
                world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
                color=carla.Color(r=255, g=0, b=0), life_time=120.0,
                persistent_lines=True)
            else:
                world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
                color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
                persistent_lines=True)
            i += 1

        def get_first_last_location(route):
            first_location = route[0][0].transform.location
            last_location = route[-1][0].transform.location
            return first_location, last_location

        first_location, last_location = get_first_last_location(route)

        # Spawn a vehicle and set it to drive to destination
        blueprint_library = world.get_blueprint_library()
        if vehicle == "Small":
            vehicle_bp = blueprint_library.filter("a2")[0]
        elif vehicle == "Truck":
            vehicle_bp = blueprint_library.filter("cybertruck")[0]
        elif vehicle == "Van":
             vehicle_bp = blueprint_library.filter("carlacola")[0]
            
        
        vehicle_actor = world.spawn_actor(vehicle_bp, carla.Transform(first_location+carla.Location(z=0.5)))
        vehicles_list = []
        vehicles_list.append(vehicle_actor)

        for vehicle in vehicles_list:

                
            vehicle_physics_control = vehicle.get_physics_control()

            if weather == "Rain":
                # Create Wheels Physics Control
                    
                front_left_wheel = carla.WheelPhysicsControl(tire_friction=0.7,max_steer_angle=70)
                front_right_wheel = carla.WheelPhysicsControl(tire_friction=0.7,max_steer_angle=70)
                rear_left_wheel = carla.WheelPhysicsControl(tire_friction=0.7,max_steer_angle=0)
                rear_right_wheel = carla.WheelPhysicsControl(tire_friction=0.7,max_steer_angle=0)
                wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
                vehicle_physics_control.wheels = wheels 
                vehicle.apply_physics_control(vehicle_physics_control)

                print("Changed grip to Rain")

            if weather == "Thunderstorm":
                front_left_wheel = carla.WheelPhysicsControl(tire_friction=0.3,max_steer_angle=70)
                front_right_wheel = carla.WheelPhysicsControl(tire_friction=0.3,max_steer_angle=70)
                rear_left_wheel = carla.WheelPhysicsControl(tire_friction=0.3,max_steer_angle=0)
                rear_right_wheel = carla.WheelPhysicsControl(tire_friction=0.3,max_steer_angle=0)
                wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
                vehicle_physics_control.wheels = wheels 
                vehicle.apply_physics_control(vehicle_physics_control)
                print("Changed grip Thunder")


        agent = BasicAgent(vehicle_actor)

        # Set the destination
        location = first_location 
        # destination = world.get_map().get_waypoint(location).transform

        agent.set_global_plan(route,stop_waypoint_creation=True, clean_queue=True)

        spectator = world.get_spectator()
        # Follow the route
        while True:
            actor = vehicle_actor
            actor_location = actor.get_location()
            actor_transform = actor.get_transform()
            actor_yaw = actor_transform.rotation.yaw
            spectator.set_transform(carla.Transform(actor_location+carla.Location(  z=10, 
                                                                                    x= - 10*math.cos(math.radians(actor_yaw)), 
                                                                                    y= - 10*math.sin(math.radians(actor_yaw))),
                                                                                    carla.Rotation(pitch= -30 ,yaw=actor_yaw)))
            if agent.done():
                break
            world.tick()
            control = agent.run_step()
            vehicle_actor.apply_control(control)

    finally:

            # Clean up the actors
            print('\ndestroying %d vehicles' % len(vehicles_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])



    
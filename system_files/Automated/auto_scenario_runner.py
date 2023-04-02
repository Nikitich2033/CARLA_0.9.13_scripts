import carla
import json
import os
import re
import time 
from agents.navigation.global_route_planner import GlobalRoutePlanner
import random
import math
from agents.navigation.basic_agent import BasicAgent
from carla import WeatherParameters
import logging


# The draw_waypoints function is used to visually display the waypoints on the map
# for debugging purposes. It draws labels for the waypoints with the specified road_id.
def draw_waypoints(waypoints, world, road_id=None, life_time=50.0):
    # Loop through the list of waypoints
    for waypoint in waypoints:
        # Check if the waypoint's road_id matches the given road_id, if specified
        if waypoint.road_id == road_id:
            # Draw a label at the waypoint's location with the text "O" 
            world.debug.draw_string(waypoint.transform.location, "O", draw_shadow=False,
                                    color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                    persistent_lines=True)


# The get_route_length function calculates the total length of a given route in meters
def get_route_length(route):
    length = 0  # Initialize the length variable
    # Loop through the route waypoints, excluding the last one
    for i in range(len(route) - 1):
        # Add the distance between the current waypoint and the next one to the length
        length += route[i][0].transform.location.distance(route[i + 1][0].transform.location)
    return length  # Return the total length of the route

def spawn_cars_along_route(world, route,vehicles_list):
                # Get the blueprint library
                blueprint_library = world.get_blueprint_library()

                # Get the car blueprints
                car_blueprints = blueprint_library.filter("vehicle.*")

                num_cars = 10

                # Spawn cars along the route
                for i in range(num_cars):
                    # Choose a random waypoint along the route
                    waypoint = random.choice(route)[0]

                    # Choose a random car blueprint
                    car_bp = random.choice(car_blueprints)

                    # Spawn the car at the waypoint location with some offset in z-axis to prevent collision with ground.
                    transform = carla.Transform(waypoint.transform.location + carla.Location(z=0.5),waypoint.transform.rotation)
                    
                    vehicle_actor = world.try_spawn_actor(car_bp, transform)
                    
                    if vehicle_actor:
                        # Set the vehicle to autopilot
                        vehicles_list.append(vehicle_actor)
                        vehicle_actor.set_autopilot(True)

def print_vehicle_info(vehicle,world):
                print("Game time: ", world.get_snapshot().timestamp.elapsed_seconds)
                print("Vehicle location: ", vehicle.get_location())
                print("Vehicle velocity: ", vehicle.get_velocity())
                print("Vehicle throttle: ", vehicle.get_control().throttle)


def save_vehicle_info(vehicle, file_path, collision_count, lane_cross_counter,world):
            # Check if file exists, create it if it doesn't
            if not os.path.exists(file_path):
                with open(file_path, 'w') as f:
                    json.dump([], f)

            # Load existing data from file
            with open(file_path, 'r') as f:
                data = json.load(f)

            # Add new data
            new_data = {
                'game_time': world.get_snapshot().timestamp.elapsed_seconds,
                'vehicle_location': {'x': vehicle.get_location().x, 'y': vehicle.get_location().y, 'z': vehicle.get_location().z},
                'vehicle_velocity': {'x': vehicle.get_velocity().x, 'y': vehicle.get_velocity().y, 'z': vehicle.get_velocity().z},
                'vehicle_throttle': vehicle.get_control().throttle,
                'unique_collisions': len(collision_count),
                'solid_lane_crosses': lane_cross_counter.get(carla.LaneMarkingType.Solid, 0),
                'double_solid_lane_crosses': lane_cross_counter.get(carla.LaneMarkingType.SolidSolid, 0)
            }

            data.append(new_data)

            # Save data to file
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=4)

#function to get the first location and last location of the route
def get_first_last_location(route):
            first_location = route[0][0].transform.location
            last_location = route[-1][0].transform.location
            return first_location, last_location


def main():
    # Open the JSON file containing the scenario data generated in step 1
    with open("step1/random_scenarios.json", "r") as file:
        scenario_data = json.load(file)

    # Initialize the directory and highest number for auto_scenario files
    directory = 'step2'
    highest_number = 0

    # Find the highest numbered auto_scenario file
    for filename in os.listdir(directory):
        match = re.search(r'auto_scenario_(\d+)', filename)
        if match:
            number = int(match.group(1))
            if number > highest_number:
                highest_number = number
    
    # Loop through the scenario data starting from the highest_number found     
    for scenario_num in range(highest_number,len(scenario_data)):
        try:
           # Extract scenario parameters from the scenario_data
            weather = scenario_data[scenario_num]["weather"]
            vehicle = scenario_data[scenario_num]["vehicle"]
            traffic = scenario_data[scenario_num]["traffic"]
            emergency = scenario_data[scenario_num]["emergency"]
            timeOfDay = scenario_data[scenario_num]["timeOfDay"]
            location = scenario_data[scenario_num]["location"]
            pedestrians = scenario_data[scenario_num]["pedestrians"]
            pedestrian_cross = scenario_data[scenario_num]["pedestrian_cross"]
            start_x = scenario_data[scenario_num]["start_x"]
            start_y = scenario_data[scenario_num]["start_y"]
            end_x = scenario_data[scenario_num]["end_x"]
            end_y = scenario_data[scenario_num]["end_y"]
            scenario_num = scenario_data[scenario_num]["scenario_num"]

            # Connect to CARLA and load the world based on the location parameter
            client = carla.Client("localhost", 2000)

            if location == "Downtown":
                world = client.load_world('Town05')
            if location == "Urban":
                world = client.load_world('Town03')
            if location == "Country":
                world = client.load_world('Town07')

            # Set up the spectator camera, map, and traffic manager
            spectator = world.get_spectator()
            map = world.get_map()

            traffic_manager = client.get_trafficmanager(8000)
            traffic_manager.set_global_distance_to_leading_vehicle(2.5)
            traffic_manager.set_synchronous_mode(True)
            traffic_manager.global_percentage_speed_difference(50.0)


            # Initialize weather and timeOfDay parameters
            cloudiness=0.0,
            precipitation=0.0,
            sun_altitude_angle=70.0  # 70 degrees is around noon

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
                precipitation_deposits=60
            
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
            

            # Initialize a GlobalRoutePlanner with the map and lane width
            grp = GlobalRoutePlanner(map,2)

            spawn_points = map.get_spawn_points()

            start_location = carla.Location(x=start_x,y=start_y)
            end_location = carla.Location(x=end_x,y=end_y)
            
            # Generate a route from the start_location to the end_location
            route = grp.trace_route(start_location,end_location)
        
                    
            # Draw the route on the map
            i = 0
            for w in route:
                if i % 10 == 0:
                    world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
                    color=carla.Color(r=255, g=0, b=0), life_time=1000.0,
                    persistent_lines=True)
                else:
                    world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
                    color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
                    persistent_lines=True)
                i += 1

            # Initialize lists to store vehicle and pedestrian actors
            vehicles_list = []
            walkers_list = []
            all_id = []

            
            first_location, last_location = get_first_last_location(route)

            
            blueprint_library = world.get_blueprint_library()
            pedestrian_bps = blueprint_library.filter("walker.pedestrian.*")

            # Spawn a vehicle based on the vehicle parameter
            if vehicle == "Small":
                vehicle_bp = blueprint_library.filter("a2")[0]
            elif vehicle == "Truck":
                vehicle_bp = blueprint_library.filter("cybertruck")[0]
            elif vehicle == "Van":
                vehicle_bp = blueprint_library.filter("carlacola")[0]
                 
            vehicle_actor = world.spawn_actor(vehicle_bp, carla.Transform(first_location+carla.Location(z=0.5)))
            
            vehicles_list.append(vehicle_actor)

            vehicle_ids = []

            # If traffic is specified, spawn additional vehicles
            if traffic == "Light" or traffic == "Heavy" or traffic == "Medium": 
                
                filtered_spawn_points = []

                # Get unique road IDs
                road_ids = list(set(waypoint[0].road_id for waypoint in route))
              
                for road_id in road_ids:
                    for point in spawn_points:
                        if map.get_waypoint(point.location).road_id == road_id:
                            filtered_spawn_points.append(point)
                       
                number_of_spawn_points = len(filtered_spawn_points)

                # Set the number of cars based on the traffic parameter
                if traffic == "Light":
                    num_cars = 15
                elif traffic == "Medium":
                    num_cars = 25
                elif traffic == "Heavy":
                    num_cars = 40
                
                
                # Prepare a batch of SpawnActor and SetAutopilot commands for the vehicles
                SpawnActor = carla.command.SpawnActor
                SetAutopilot = carla.command.SetAutopilot
                FutureActor = carla.command.FutureActor

                blueprints = world.get_blueprint_library().filter('vehicle.*')
                blueprints = sorted(blueprints, key=lambda bp: bp.id)

                if num_cars < number_of_spawn_points:
                    random.shuffle(filtered_spawn_points)
                elif num_cars > number_of_spawn_points:
                    msg = 'requested %d vehicles, but could only find %d spawn points'
                    logging.warning(msg, num_cars, number_of_spawn_points)
                    num_cars = number_of_spawn_points

                # Apply the batch of commands and store the vehicles in the vehicles_list
                batch = []
                for n, transform in enumerate(filtered_spawn_points):
                    if n >= num_cars:
                        break
                    blueprint = random.choice(blueprints) 
                    if blueprint.has_attribute('color'):
                        color = random.choice(blueprint.get_attribute('color').recommended_values)
                        blueprint.set_attribute('color', color)
                    if blueprint.has_attribute('driver_id'):
                        driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                        blueprint.set_attribute('driver_id', driver_id)

                    blueprint.set_attribute('role_name', 'autopilot')

                    batch.append(SpawnActor(blueprint, transform)
                        .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

             
                for response in client.apply_batch_sync(batch):
                    if response.error:
                        print(f"Error: {response.error}")
                    else:
                        vehicles_list.append(world.get_actor(response.actor_id))
                        vehicle_ids.append(response.actor_id)
                         
                # Spawn additional cars along the route if needed
                spawn_cars_along_route(world,route,vehicles_list)
                    
                # for response in client.apply_batch_sync(batch, True):
                #     if response.error:
                #         logging.error(response.error)
                #     else:
                #         vehicle_ids.append(response.actor_id)
                #         vehicles_list.append(world.get_actor(response.actor_id))
     

            if pedestrians:
                # Set the percentage of pedestrians running and crossing based on the emergency parameter
                if emergency == "No":
                    percentagePedestriansRunning = 20
                    percentagePedestriansCrossing = 30
                else:
                    percentagePedestriansRunning = 80
                    percentagePedestriansCrossing = 70

                # Generate random spawn points for pedestrians
                spawn_points = []
                for _ in range(30):
                    loc = world.get_random_location_from_navigation()
                    if loc:
                        spawn_point = carla.Transform()
                        spawn_point.location = loc
                        spawn_points.append(spawn_point)

                # Spawn pedestrian actors
                batch = []
                walker_speeds = []
                for spawn_point in spawn_points:
                    walker_bp = random.choice(pedestrian_bps)
                    # Set pedestrian attributes
                    if walker_bp.has_attribute('is_invincible'):
                        walker_bp.set_attribute('is_invincible', 'false')
                    if walker_bp.has_attribute('speed'):
                        if random.random() > percentagePedestriansRunning:
                            walker_speeds.append(walker_bp.get_attribute('speed').recommended_values[1])  # Walking
                        else:
                            walker_speeds.append(walker_bp.get_attribute('speed').recommended_values[2])  # Running
                    else:
                        print("Walker has no speed")
                        walker_speeds.append(0.0)
                    batch.append(SpawnActor(walker_bp, spawn_point))

                results = client.apply_batch_sync(batch, True)

                # Store spawned walkers
                walkers_list = []
                for i, result in enumerate(results):
                    if result.error:
                        logging.error(result.error)
                    else:
                        walkers_list.append({"id": result.actor_id, "speed": walker_speeds[i]})

                # Spawn walker controllers
                batch = []
                walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
                for walker in walkers_list:
                    batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walker["id"]))

                results = client.apply_batch_sync(batch, True)

                # Store walker controllers
                for i, result in enumerate(results):
                    if result.error:
                        logging.error(result.error)
                    else:
                        walkers_list[i]["con"] = result.actor_id

                # Get pedestrian and controller actors
                all_id = [actor_id for walker in walkers_list for actor_id in [walker["con"], walker["id"]]]
                all_actors = world.get_actors(all_id)

                # Set the percentage of pedestrians that can cross the road
                if pedestrian_cross:
                    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)

                # Control pedestrian behavior
                for i in range(0, len(all_id), 2):
                    controller = all_actors[i]
                    controller.start()
                    controller.go_to_location(world.get_random_location_from_navigation())
                    controller.set_max_speed(float(walkers_list[i // 2]["speed"]))

            # Print the number of spawned vehicles and walkers
            print(f'spawned {len(vehicle_ids) + len(vehicles_list)} vehicles and {len(walkers_list)} walkers, press Ctrl+C to exit.')
   


            for vehicle in vehicles_list:
                # Turn on front lights for the vehicle
                light_state = carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam
                vehicle.set_light_state(carla.VehicleLightState(light_state))

                # Get the vehicle's physics control
                vehicle_physics_control = vehicle.get_physics_control()

                # Adjust the vehicle's tire friction based on weather conditions
                if weather == "Rain":
                    # Create Wheels Physics Control for rain
                    front_left_wheel = carla.WheelPhysicsControl(tire_friction=0.9, max_steer_angle=70)
                    front_right_wheel = carla.WheelPhysicsControl(tire_friction=0.9, max_steer_angle=70)
                    rear_left_wheel = carla.WheelPhysicsControl(tire_friction=0.9, max_steer_angle=0)
                    rear_right_wheel = carla.WheelPhysicsControl(tire_friction=0.9, max_steer_angle=0)
                    wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
                    vehicle_physics_control.wheels = wheels
                    vehicle.apply_physics_control(vehicle_physics_control)

                if weather == "Thunderstorm":
                    # Create Wheels Physics Control for thunderstorm
                    front_left_wheel = carla.WheelPhysicsControl(tire_friction=0.75, max_steer_angle=70)
                    front_right_wheel = carla.WheelPhysicsControl(tire_friction=0.75, max_steer_angle=70)
                    rear_left_wheel = carla.WheelPhysicsControl(tire_friction=0.75, max_steer_angle=0)
                    rear_right_wheel = carla.WheelPhysicsControl(tire_friction=0.75, max_steer_angle=0)
                    wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
                    vehicle_physics_control.wheels = wheels
                    vehicle.apply_physics_control(vehicle_physics_control)

                   
            agent = BasicAgent(vehicle_actor)

          
            # Set the global plan for the agent to follow the route
            agent.set_global_plan(route, stop_waypoint_creation=True, clean_queue=True)

            sensors = []

            # Set up a collision sensor
            collision_bp = world.get_blueprint_library().find('sensor.other.collision')
            collision_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
            collision_sensor = world.spawn_actor(collision_bp, collision_transform, attach_to=vehicle_actor)

            collision_count = {}
            max_collisions = 1500
            collided_actors = set()

            # Define the callback function for the collision sensor
            def on_collision(event):
                other_actor = event.other_actor
                if other_actor not in collided_actors:
                    print("Collision with {}".format(other_actor.type_id))
                    collided_actors.add(other_actor)
                    if other_actor.type_id not in collision_count:
                        collision_count[other_actor.type_id] = 0
                    collision_count[other_actor.type_id] += 1

            # Subscribe the collision sensor to the callback function
            collision_sensor.listen(on_collision)

            # Set up a lane invasion sensor
            lane_invasion_bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            lane_invasion_transform = carla.Transform()
            lane_invasion_sensor = world.spawn_actor(lane_invasion_bp, lane_invasion_transform, attach_to=vehicle_actor)

            lane_cross_counter = {}

            # Define the callback function for the lane invasion sensor
            def on_lane_invasion(event):
                for marking in event.crossed_lane_markings:
                    if marking.type == carla.LaneMarkingType.Solid or marking.type == carla.LaneMarkingType.SolidSolid:
                        if marking.type not in lane_cross_counter:
                            lane_cross_counter[marking.type] = 0
                        lane_cross_counter[marking.type] += 1
                        print(f"Crossed: {marking.type} - Count: {lane_cross_counter[marking.type]}")

            # Subscribe the lane invasion sensor to the callback function
            lane_invasion_sensor.listen(on_lane_invasion)

            # Add the sensors to the list
            sensors.append(collision_sensor)
            sensors.append(lane_invasion_sensor)

            # Initialize the output file for the scenario
            file_path = f'step2/auto_scenario_{scenario_num}.json'
            with open(file_path, 'w') as f:
                json.dump([], f)

            # Initialize variables for velocity and time
            velocity_check = time.time()
            info_time = world.get_snapshot().timestamp.elapsed_seconds

            while True:
                # Set the spectator camera to follow the vehicle from above and behind
                actor = vehicle_actor
                actor_location = actor.get_location()
                actor_transform = actor.get_transform()
                actor_yaw = actor_transform.rotation.yaw
                spectator.set_transform(carla.Transform(actor_location + carla.Location(z=10,
                                                                                        x=-10 * math.cos(math.radians(actor_yaw)),
                                                                                        y=-10 * math.sin(math.radians(actor_yaw))),
                                                        carla.Rotation(pitch=-30, yaw=actor_yaw)))

                # Check if the vehicle has reached the maximum allowed number of collisions
                if any(count > max_collisions for count in collision_count.values()):
                    print('Vehicle stuck detected, stopping scenario')
                    break

                # Check if the vehicle is stationary for more than 50 seconds
                velocity = vehicle_actor.get_velocity()
                if math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) < 0.1:
                    if time.time() - velocity_check > 50:
                        print("Vehicle has been stationary for more than 50 seconds")
                        break
                    else:
                        velocity_check = time.time()

                # Check if the agent has completed the route
                if agent.done():
                    break

                # Save vehicle information every second
                if world.get_snapshot().timestamp.elapsed_seconds - info_time >= 1:
                    save_vehicle_info(vehicle_actor, file_path, collision_count, lane_cross_counter, world)
                    info_time = world.get_snapshot().timestamp.elapsed_seconds

                # Apply agent control to the vehicle
                control = agent.run_step()
                vehicle_actor.apply_control(control)
                world.tick()

        finally:
            # Clean up the actors
            print('\ndestroying %d vehicles' % (len(vehicle_ids) + len(vehicles_list)))
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_ids])
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

            # Stop all actors
            for i in range(0, len(all_id), 2):
                all_actors[i].stop()

            # Destroy the sensors
            for i in range(len(sensors) - 1):
                sensors[i].destroy()

            # Clean up the walkers
            print('\ndestroying %d walkers' % len(walkers_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
            time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')





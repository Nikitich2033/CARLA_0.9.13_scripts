try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_b
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_n
    from pygame.locals import K_o
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_t
    from pygame.locals import K_v
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')
import carla
import logging
import math
import random
import json
import os
import time 
from agents.navigation.global_route_planner import GlobalRoutePlanner
import random
import math
from carla import WeatherParameters
import logging
from carla import VehicleLightState as vls

 # Open the JSON file
with open("user_input/scenarios.json", "r") as file:
    scenario_data = json.load(file)

# Initialize variables with the values from the JSON file

def main():
    
    for scenario_num in range(len(scenario_data)):
        try:

            scenario_number = scenario_data[scenario_num]["scenario_num"]
            weather = scenario_data[scenario_num]["weather"]
            intersections = scenario_data[scenario_num]["intersections"]
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
            route_length = scenario_data[scenario_num]["route_length"]
            total_difficulty_rating = scenario_data[scenario_num]["total_difficulty_rating"]

            # Print the values of the variables

            print("Scenario number: ", scenario_number)
            print("Weather: ", weather)
            print("Num of Intersections: ", intersections)
            print("Vehicle: ", vehicle)
            print("Traffic: ", traffic)
            print("Emergency: ", emergency)
            print("Time: ", timeOfDay)
            print("Location: ", location)
            print("Pedestrians: ", pedestrians)
            print("Pedestrian cross: ", pedestrian_cross)
            print("Route length: ", route_length)
            print("Total difficulty: ",total_difficulty_rating)

            client = carla.Client("localhost", 2000)

            if location == "Downtown":
                world = client.load_world('Town05')
            if location == "Urban":
                world = client.load_world('Town03')
            if location == "Country":
                world = client.load_world('Town07')

            spectator = world.get_spectator()
            map = world.get_map()


            original_settings = world.get_settings()
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

            traffic_manager.set_global_distance_to_leading_vehicle(2.5)
            traffic_manager.global_percentage_speed_difference(50.0)

            cloudiness=0.0,
            precipitation=0.0,
            sun_altitude_angle=70.0  # 70 degrees is around noon

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

    
            grp = GlobalRoutePlanner(map,2)

            spawn_points = map.get_spawn_points()

            # Set the desired number of junctions
            num_junctions = intersections

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
            
            start_location = carla.Location(x=start_x,y=start_y)
            end_location = carla.Location(x=end_x,y=end_y)
            
            # print(start_waypoint.transform.location.distance(end_waypoint.transform.location))
            route = grp.trace_route(start_location,end_location)
        
                    
            # The 'waypoints' variable now contains a list of waypoints that define a route between 'start_pose' and 'end_pose' that goes through 'num_junctions' junctions.

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

            vehicles_list = []
            walkers_list = []
            all_id = []

            def spawn_random_pedestrians_and_cars(world, route):
                # Get the blueprint library
                blueprint_library = world.get_blueprint_library()

                # Get the pedestrian and car blueprints
                pedestrian_blueprints = blueprint_library.filter("walker.pedestrian.*")
                car_blueprints = blueprint_library.filter("vehicle.*")

                # Set the number of pedestrians and cars to spawn
                num_pedestrians = 0
                num_cars = 10

                # Spawn pedestrians along the route
                for i in range(num_pedestrians):
                    # Choose a random waypoint along the route
                    waypoint = random.choice(route)[0]

                    # Choose a random pedestrian blueprint
                    pedestrian_bp = random.choice(pedestrian_blueprints)

                    # Spawn the pedestrian at the waypoint location
                    world.try_spawn_actor(pedestrian_bp, waypoint.transform)

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

            def get_first_last_location(route):
                first_location = route[0][0].transform.location
                last_location = route[-1][0].transform.location
                return first_location, last_location

            
            first_location, last_location = get_first_last_location(route)

            # Spawn a vehicle and set it to drive to destination
            blueprint_library = world.get_blueprint_library()
            pedestrian_bps = blueprint_library.filter("walker.pedestrian.*")

            if vehicle == "Small":
                vehicle_bp = blueprint_library.filter("a2")[0]
            elif vehicle == "Truck":
                vehicle_bp = blueprint_library.filter("cybertruck")[0]
            elif vehicle == "Van":
                vehicle_bp = blueprint_library.filter("carlacola")[0]
                 
            vehicle_actor = world.spawn_actor(vehicle_bp, carla.Transform(first_location+carla.Location(z=0.5)))
            
            vehicles_list.append(vehicle_actor)

            vehicle_ids = []
            if traffic == "Light" or traffic == "Heavy" or traffic == "Medium": 
                
                filtered_spawn_points = []

                # Get unique road IDs
                road_ids = list(set(waypoint[0].road_id for waypoint in route))
              
                for road_id in road_ids:
                    # waypoints = map.generate_waypoints(2.0)
                    for point in spawn_points:
                        if map.get_waypoint(point.location).road_id == road_id:
                            filtered_spawn_points.append(point)
                       
                number_of_spawn_points = len(filtered_spawn_points)

                if traffic == "Light":
                    num_cars = 15
                elif traffic == "Medium":
                    num_cars = 25
                elif traffic == "Heavy":
                    num_cars = 40
                
                SpawnActor = carla.command.SpawnActor
                SetAutopilot = carla.command.SetAutopilot
                SetVehicleLightState = carla.command.SetVehicleLightState
                FutureActor = carla.command.FutureActor

                blueprints = world.get_blueprint_library().filter('vehicle.*')
                blueprints = sorted(blueprints, key=lambda bp: bp.id)

                if num_cars < number_of_spawn_points:
                    random.shuffle(filtered_spawn_points)
                elif num_cars > number_of_spawn_points:
                    msg = 'requested %d vehicles, but could only find %d spawn points'
                    logging.warning(msg, num_cars, number_of_spawn_points)
                    num_cars = number_of_spawn_points


                batch = []
                for n, transform in enumerate(filtered_spawn_points):
                    # print(transform)
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

                    # prepare the light state of the cars to spawn
                    light_state = vls.NONE
                    if True:
                        light_state = vls.Position | vls.LowBeam | vls.LowBeam

                    # spawn the cars and set their autopilot and light state all together
                    batch.append(SpawnActor(blueprint, transform)
                        .then(SetAutopilot(FutureActor, True, traffic_manager.get_port()))
                        .then(SetVehicleLightState(FutureActor, light_state)))

                spawn_random_pedestrians_and_cars(world,route)  
                    
                for response in client.apply_batch_sync(batch, True):
                    if response.error:
                        logging.error(response.error)
                    else:
                        vehicle_ids.append(response.actor_id)

            if pedestrians == True:
                if emergency == "No":
                    percentagePedestriansRunning = 20     # how many pedestrians will run
                    percentagePedestriansCrossing = 30     # how many pedestrians will walk through the road
                else:
                    percentagePedestriansRunning = 80
                    percentagePedestriansCrossing = 70
                
                # 1. take all the random locations to spawn
                spawn_points = []
                for i in range(30):
                    spawn_point = carla.Transform()
                    loc = world.get_random_location_from_navigation()
                    if (loc != None):
                        spawn_point.location = loc
                        spawn_points.append(spawn_point)
                # 2. we spawn the walker object
                batch = []
                walker_speed = []
                for spawn_point in spawn_points:
                    walker_bp = random.choice(pedestrian_bps)
                    # set as not invincible
                    if walker_bp.has_attribute('is_invincible'):
                        walker_bp.set_attribute('is_invincible', 'false')
                    # set the max speed
                    if walker_bp.has_attribute('speed'):
                        if (random.random() > percentagePedestriansRunning):
                            # walking
                            walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                        else:
                            # running
                            walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
                    else:
                        print("Walker has no speed")
                        walker_speed.append(0.0)
                    batch.append(SpawnActor(walker_bp, spawn_point))
                results = client.apply_batch_sync(batch, True)
                walker_speed2 = []
                for i in range(len(results)):
                    if results[i].error:
                        logging.error(results[i].error)
                    else:
                        walkers_list.append({"id": results[i].actor_id})
                        walker_speed2.append(walker_speed[i])
                walker_speed = walker_speed2
                # 3. we spawn the walker controller
                batch = []
                walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
                for i in range(len(walkers_list)):
                    batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
                results = client.apply_batch_sync(batch, True)
                for i in range(len(results)):
                    if results[i].error:
                        logging.error(results[i].error)
                    else:
                        walkers_list[i]["con"] = results[i].actor_id
                # 4. we put together the walkers and controllers id to get the objects from their id
                for i in range(len(walkers_list)):
                    all_id.append(walkers_list[i]["con"])
                    all_id.append(walkers_list[i]["id"])
                all_actors = world.get_actors(all_id)

                # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
                # set how many pedestrians can cross the road
                world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
                for i in range(0, len(all_id), 2):
                    # start walker
                    all_actors[i].start()
                    # set walk to random point
                    all_actors[i].go_to_location(world.get_random_location_from_navigation())
                    # max speed
                    all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

               
            print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicle_ids)+len(vehicles_list), len(walkers_list)))         

            for vehicle in vehicles_list:

                vehicle_physics_control = vehicle.get_physics_control()

                if weather == "Rain":
                    # Create Wheels Physics Control
                        
                    front_left_wheel = carla.WheelPhysicsControl(tire_friction=0.9,max_steer_angle=70)
                    front_right_wheel = carla.WheelPhysicsControl(tire_friction=0.9,max_steer_angle=70)
                    rear_left_wheel = carla.WheelPhysicsControl(tire_friction=0.9,max_steer_angle=0)
                    rear_right_wheel = carla.WheelPhysicsControl(tire_friction=0.9,max_steer_angle=0)
                    wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
                    vehicle_physics_control.wheels = wheels 
                    vehicle.apply_physics_control(vehicle_physics_control)

                    print("Changed grip to Rain")

                if weather == "Thunderstorm":
                    front_left_wheel = carla.WheelPhysicsControl(tire_friction=0.75,max_steer_angle=70)
                    front_right_wheel = carla.WheelPhysicsControl(tire_friction=0.75,max_steer_angle=70)
                    rear_left_wheel = carla.WheelPhysicsControl(tire_friction=0.75,max_steer_angle=0)
                    rear_right_wheel = carla.WheelPhysicsControl(tire_friction=0.75,max_steer_angle=0)
                    wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
                    vehicle_physics_control.wheels = wheels 
                    vehicle.apply_physics_control(vehicle_physics_control)
                    print("Changed grip Thunder")

            def print_vehicle_info(vehicle):
                    print("Game time: ", world.get_snapshot().timestamp.elapsed_seconds)
                    print("Vehicle location: ", vehicle.get_location())
                    print("Vehicle velocity: ", vehicle.get_velocity())
                    print("Vehicle throttle: ", vehicle.get_control().throttle)

            def save_vehicle_info(vehicle, file_path):

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
                        'vehicle_throttle': vehicle.get_control().throttle
                    }
                    
                    data.append(new_data)
        
                    # Save data to file
                    with open(file_path, 'w') as f:
                        json.dump(data, f,indent=4)


            pygame.init()
            display = pygame.display.set_mode((1280, 720)) 
            # Set up the clock for a decent framerate
            clock = pygame.time.Clock()


            # Set up the RGB camera
            camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
            camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
            # Set image size
            camera_bp.set_attribute('image_size_x', '1280')
            camera_bp.set_attribute('image_size_y', '720')
            # Get image size
            width = int(camera_bp.get_attribute('image_size_x'))
            height = int(camera_bp.get_attribute('image_size_y'))
            camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle_actor)

            # Function to convert image data from the camera sensor to a Pygame surface
            def process_image(image):
                image.convert(carla.ColorConverter.Raw)
                array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
                array = np.reshape(array, (image.height, image.width, 4))
                array = array[:, :, :3]
                array = array[:, :, ::-1]
                surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
                 
                return surface
            # Set up a callback function for when the camera receives an image
            camera.listen(lambda image: display.blit(process_image(image), (0, 0)))
            

            sensors = []

            collision_bp = world.get_blueprint_library().find('sensor.other.collision')
            collision_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
            collision_sensor = world.spawn_actor(collision_bp, collision_transform, attach_to=vehicle_actor)

            def on_collision(event):
                # Do something when a collision occurs
                print("Collision detected:", event)

            collision_sensor.listen(on_collision)


            # Get the blueprint for the lane invasion sensor
            lane_invasion_bp = world.get_blueprint_library().find('sensor.other.lane_invasion')

            # Create a transform object to specify the location and rotation of the sensor relative to the vehicle
            lane_invasion_transform = carla.Transform()

            # Spawn the lane invasion sensor actor and attach it to the vehicle
            lane_invasion_sensor = world.spawn_actor(lane_invasion_bp, lane_invasion_transform, attach_to=vehicle_actor)

            # Set up a callback function to handle lane invasion events
            def on_lane_invasion(event):
                # Do something when a lane invasion occurs
                for marking in event.crossed_lane_markings:
                    print(f"Crossed: {marking.type}")

            lane_invasion_sensor.listen(on_lane_invasion)

            sensors.append(collision_sensor)
            sensors.append(lane_invasion_sensor)

            info_time = world.get_snapshot().timestamp.elapsed_seconds
            file_path = f'user_input/manual_scenario_{scenario_num}.json'
            with open(file_path, 'w') as f:
                        json.dump([], f)

            key_press_times = {}
            while True:

                actor = vehicle_actor
                actor_location = actor.get_location()
                actor_transform = actor.get_transform()
                actor_yaw = actor_transform.rotation.yaw
                spectator.set_transform(carla.Transform(actor_location+carla.Location(  z=10, 
                                                                                        x= - 10*math.cos(math.radians(actor_yaw)), 
                                                                                        y= - 10*math.sin(math.radians(actor_yaw))),
                
                                                                                         carla.Rotation(pitch= -30 ,yaw=actor_yaw)))
                world.tick()

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return

                # Set vehicle control based on keys pressed
                # Get pressed keys
                keys = pygame.key.get_pressed()

                control = carla.VehicleControl()
                if keys[pygame.K_UP]:
                    control.throttle = 0.65
                if keys[pygame.K_LEFT]:
                    control.steer = -0.5
                if keys[pygame.K_RIGHT]:
                    control.steer = 0.5
                if keys[pygame.K_SPACE]:
                    control.brake = 0.75
                if keys[pygame.K_DOWN]:
                    control.reverse = True
                    control.throttle = 0.65

                # Apply vehicle control to the vehicle object
                vehicle_actor.apply_control(control)
                distance = last_location.distance(vehicle_actor.get_location())
                # print(distance)
                if distance <= 1:
                    
                    break

                if world.get_snapshot().timestamp.elapsed_seconds - info_time >= 1:
                        
                        save_vehicle_info(vehicle_actor, file_path)
                        # print_vehicle_info(vehicle)
                        info_time = world.get_snapshot().timestamp.elapsed_seconds

                clock.tick(60)
                pygame.display.flip()

                

        finally:

            # Clean up the actors
            print('\ndestroying %d vehicles' % (len(vehicle_ids)+len(vehicles_list)))
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_ids])
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
            
            # stop walker controllers (list is [controller, actor, controller, actor ...])
            for i in range(0, len(all_id), 2):
                all_actors[i].stop()
            
            for i in range(len(sensors)-1):
                sensors[i].destroy()

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
    
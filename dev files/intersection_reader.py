import json
import os
from carla import VehicleLightState as vls
import logging 
import time
import math
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner
import carla
from carla import WeatherParameters

# Create a new CARLA client
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)


# Create a recorder and start it
client.start_recorder("~/carla_0.9.10.1/PythonAPI/examples/additions/test.log", True)

# Load the scenario
world = client.load_world('Town03')


def main():

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    # Open the JSON file
    with open("user_input/scenarios.json", "r") as file:
        scenario_data = json.load(file)

    # Initialize variables with the values from the JSON file

    for scenario_num in range(len(scenario_data)):
        print("new scenario: "+ str(scenario_num))
        weather = scenario_data[scenario_num]["weather"]
        intersection = scenario_data[scenario_num]["intersection"]
        num_cars = scenario_data[scenario_num]["num_cars"]
        road = scenario_data[scenario_num]["road"]
        vehicle = scenario_data[scenario_num]["vehicle"]
        traffic = scenario_data[scenario_num]["traffic"]
        emergency = scenario_data[scenario_num]["emergency"]
        timeOfDay = scenario_data[scenario_num]["timeOfDay"]
        location = scenario_data[scenario_num]["location"]
        intersection = scenario_data[scenario_num]["intersection"]
        pedestrians = scenario_data[scenario_num]["pedestrians"]
        pedestrian_cross = scenario_data[scenario_num]["pedestrian_cross"]

        # Print the values of the variables
  
        print("Weather: ", weather)
        print("Intersection: ", intersection)
        print("Number of cars: ", num_cars)
        print("Road: ", road)
        print("Vehicle: ", vehicle)
        print("Traffic: ", traffic)
        print("Emergency: ", emergency)
        print("Time: ", timeOfDay)
        print("Location: ", location)
        print("Intersection: ", intersection)
        print("Pedestrians: ", pedestrians)
        print("Pedestrian cross: ", pedestrian_cross)


        # Get the spectator camera
        spectator = world.get_spectator()

        vehicles_list = []
        walkers_list = []
        all_id = []
        followed_vehicle = None


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


       

        weather_params = WeatherParameters(
            cloudiness=cloudiness,
            precipitation=precipitation,
            sun_altitude_angle=sun_altitude_angle,  
            precipitation_deposits=precipitation_deposits 
        )

        # Set the weather in the simulation
        world.set_weather(weather_params)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)


        # Spawn the specified number of cars
        try:
            
            # Get the blueprint for the pedestrian and set its attributes
            blueprint_library = world.get_blueprint_library()
        

            # # Get the blueprint for a Cybertruck
            cybertruck_bp = world.get_blueprint_library().find('vehicle.tesla.cybertruck')

    
            bad_actor_bp = world.get_blueprint_library().find('vehicle.audi.etron')
            bad_transform = carla.Transform(carla.Location(x= 13, y=130, z=3.5), carla.Rotation(yaw=180))
            bad_car = world.spawn_actor(bad_actor_bp, bad_transform)


            map = world.get_map()

           
            #draws waypoints for a certain road 
            def draw_waypoints(waypoints, road_id=None, life_time=50.0):
                spawned = False 
                for waypoint in waypoints:
                    
                    if(waypoint.road_id == road_id):
                        world.debug.draw_string(waypoint.transform.location, str(road_id), draw_shadow=False,
                                                color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                                persistent_lines=True)

            waypoints = client.get_world().get_map().generate_waypoints(distance=1.0)
            # draw_all_waypoints(waypoints)
            # draw_waypoints(waypoints, road_id=24, life_time=100)
            # draw_waypoints(waypoints, road_id=30, life_time=100)
            # draw_waypoints(waypoints, road_id=110, life_time=100)


            filtered_waypoints = []
            for waypoint in waypoints:
                if(waypoint.road_id == 24):
                    filtered_waypoints.append(waypoint)

            spawn_point = filtered_waypoints[10].transform
            spawn_point.location.z += 2

            filtered_waypoints = []
            for waypoint in waypoints:
                if(waypoint.road_id == 30):
                    filtered_waypoints.append(waypoint)

            end_point = filtered_waypoints[10].transform
            end_point.location.z += 2

             # dao = GlobalRoutePlannerDAO(amap, sampling_resolution)
            grp = GlobalRoutePlanner(map,2)
            # grp.setup()
           
            a = spawn_point.location
            b = end_point.location

            w1 = grp.trace_route(a, b) # there are other funcations can be used to generate a route in GlobalRoutePlanner.

            i = 0
            for w in w1:
                if i % 10 == 0:
                    world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
                    color=carla.Color(r=255, g=0, b=0), life_time=120.0,
                    persistent_lines=True)
                else:
                    world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
                    color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
                    persistent_lines=True)
                i += 1



            cybertruck = client.get_world().spawn_actor(cybertruck_bp, spawn_point)
            vehicles_list.append(cybertruck)
            vehicles_list.append(bad_car)

            cybertruck_agent = BasicAgent(cybertruck)
            
            
            
            # Get the spectator camera
            spectator = world.get_spectator()

            # Get the position of an actor in the scene
            actor = cybertruck
            actor_location = actor.get_location()

            # Set the camera to look at the actor
            spectator.set_transform(carla.Transform(actor_location+carla.Location(z=40), carla.Rotation(pitch=-90)))

            target_road_waypoints = []
            for waypoint in waypoints:
                if(waypoint.road_id == 30):
                    target_road_waypoints.append(waypoint)

            # target_waypoint = target_road_waypoints[len(target_road_waypoints)-1].transform

            cybertruck_agent.set_global_plan(w1,stop_waypoint_creation=True, clean_queue=True)

            # client.get_world().debug.draw_string(target_waypoint.location, 
            #                                         'O', draw_shadow=False,
            #                                         color=carla.Color(r=255, g=0, b=0), life_time=0,
            #                                         persistent_lines=True)


            # Spawn pedestrians if specified
            if pedestrians == True:
                
                pedestrian_bp = blueprint_library.find('walker.pedestrian.0001')
                pedestrian_bp.set_attribute('is_invincible', 'false')
                pedestrian_bp.set_attribute('speed', '20.0')

                pedestrian = world.spawn_actor(pedestrian_bp, carla.Transform(carla.Location(x=16, y=134, z=3.5),carla.Rotation(yaw=180)))
                pedestrian_control = carla.WalkerControl()
                pedestrian_control.speed = 20
                pedestrian_control.direction = carla.Vector3D(x=-1.0, y=0.0, z=0.0)
                pedestrian.apply_control(pedestrian_control)

                world.tick()


                # example of how to use parameters
               
                walkers_list.append(pedestrian)

            print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))
            
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
        

            def print_vehicle_info(vehicle):
                print("Game time: ", world.get_snapshot().timestamp.elapsed_seconds)
                print("Vehicle location: ", vehicle.get_location())
                print("Vehicle velocity: ", vehicle.get_velocity())
                print("Vehicle throttle: ", vehicle.get_control().throttle)
            
            def save_vehicle_info(scenario_num, vehicle, folder):
                
               
                file_path = f'{folder}/scenario_{scenario_num}.json'
                
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

            t_end = time.time() + 60
            info_time = world.get_snapshot().timestamp.elapsed_seconds
            while time.time() < t_end:
                actor = cybertruck
                actor_location = actor.get_location()
                actor_transform = actor.get_transform()
                actor_yaw = actor_transform.rotation.yaw
                spectator.set_transform(carla.Transform(actor_location+carla.Location(  z=10, 
                                                                                        x= - 10*math.cos(math.radians(actor_yaw)), 
                                                                                        y= - 10*math.sin(math.radians(actor_yaw))),
                                                                                        carla.Rotation(pitch= -30 ,yaw=actor_yaw)))
                
                distance = actor_location.distance(bad_car.get_location())
                # print(f"The distance between actor 1 and actor 2 is {distance:.2f} meters.")

                if cybertruck.is_at_traffic_light():
                    traffic_light = cybertruck.get_traffic_light()
                    # print(traffic_light)
                    if traffic_light.get_state() == carla.TrafficLightState.Red:
                        # world.hud.notification("Traffic light changed! Good to go!")
                        traffic_light.set_state(carla.TrafficLightState.Green)

               
                if distance < 35:
            
                    throttle = 1.0
                    control = carla.VehicleControl(throttle=throttle)
                    bad_car.apply_control(control)

                if cybertruck_agent.done():
                    print("The target has been reached, stopping the simulation")
                    break

                cybertruck.apply_control(cybertruck_agent.run_step())

                 
                # Wait for one second
                if world.get_snapshot().timestamp.elapsed_seconds - info_time >= 1:
                    file_path = 'user_input'
                    save_vehicle_info(scenario_num, cybertruck, file_path)
                    print_vehicle_info(cybertruck)
                    info_time = world.get_snapshot().timestamp.elapsed_seconds
               
                world.tick()

    
            # Wait for the user to end the script
        finally:

            # Clean up the actors
            print('\ndestroying %d vehicles' % len(vehicles_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

            # # stop walker controllers (list is [controller, actor, controller, actor ...])
            for i in range(0, len(all_id), 2):
                all_actors[i].stop()

            print('\ndestroying %d walkers' % len(walkers_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
            
            client.stop_recorder()

            time.sleep(0.5)
            client.load_world('Town03')

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')

#AI search heuristics/ end game database 
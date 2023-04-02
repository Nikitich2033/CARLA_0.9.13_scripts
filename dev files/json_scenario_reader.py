import json
from carla import VehicleLightState as vls
import logging 
import time
import math


import carla
import random

def main():

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    # Open the JSON file
    with open("user_input/scenario_data.json", "r") as file:
        scenario_data = json.load(file)

    # Initialize variables with the values from the JSON file
    num_scenarios = scenario_data["num_scenarios"]
    weather = scenario_data["weather"]
    pedestrians = scenario_data["pedestrians"]
    intersection = scenario_data["intersection"]
    num_cars = scenario_data["num_cars"]

    # Print the values of the variables
    print("Number of scenarios: ", num_scenarios)
    print("Weather: ", weather)
    print("Pedestrians: ", pedestrians)
    print("Intersection: ", intersection)
    print("Number of cars: ", num_cars)

    # Connect to the CARLA server
    client = carla.Client("localhost", 2000)
    client.set_timeout(50.0)
    world = client.get_world()


    # Get the spectator camera
    spectator = world.get_spectator()

    vehicles_list = []
    walkers_list = []
    all_id = []
    followed_vehicle = None

    # Set the weather conditions
    if weather == "Sunny":
        world.set_weather(carla.WeatherParameters.ClearNoon)
    elif weather == "Rain":
        world.set_weather(carla.WeatherParameters.MidRainyNoon)
    elif weather == "Thunderstorm":
        world.set_weather(carla.WeatherParameters.HardRainNoon)

    # Spawn the actor car
    # bp = world.get_blueprint_library().filter("vehicle.audi.a2")[0]
    # spawn_point = random.choice(world.get_map().get_spawn_points())
    # actor_car = world.spawn_actor(bp, spawn_point)

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_global_distance_to_leading_vehicle(1.0)
  

    # Spawn the specified number of cars
    try:
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)


        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor

        blueprints = world.get_blueprint_library().filter('vehicle.*')
        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        if num_cars < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif num_cars > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, num_cars, number_of_spawn_points)
            num_cars = number_of_spawn_points

        batch = []
        for n, transform in enumerate(spawn_points):
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
            
            

            for response in client.apply_batch_sync(batch, True):
                if response.error:
                    logging.error(response.error)
                else:
                    vehicles_list.append(response.actor_id)
                    followed_vehicle_id = response.actor_id
            
    
            # actor_location = world.get_actor(vehicles_list[0]).get_location()
            # spectator.set_transform(carla.Transform(actor_location+carla.Location(z=40), carla.Rotation(pitch=-60)))

        # for i in range(num_cars):
        #     bp = world.get_blueprint_library().filter("vehicle.*")[0]
        #     spawn_point = random.choice(world.get_map().get_spawn_points())
        #     print(spawn_point)
        #     car = world.spawn_actor(bp, spawn_point)

        # Spawn pedestrians if specified
        if pedestrians == "Yes":
            # bp = world.get_blueprint_library().filter("walker.pedestrian.*")[0]
            # spawn_points = world.get_map().get_spawn_points()
            # for spawn_point in spawn_points:
            #     # if spawn_point.is_on_offroad:
            #     pedestrian = world.spawn_actor(bp, spawn_point)
            
            blueprintsWalkers = world.get_blueprint_library().filter("walker.pedestrian.*")
            percentagePedestriansRunning = 0.0      # how many pedestrians will run
            percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
            # 1. take all the random locations to spawn
            spawn_points = []
            number_of_walkers = 20
            for i in range(number_of_walkers):
                spawn_point = carla.Transform()
                loc = world.get_random_location_from_navigation()
                if (loc != None):
                    spawn_point.location = loc
                    spawn_points.append(spawn_point)
            # 2. we spawn the walker object
            batch = []
            walker_speed = []
            for spawn_point in spawn_points:
                walker_bp = random.choice(blueprintsWalkers)
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
            # 4. we put altogether the walkers and controllers id to get the objects from their id
            for i in range(len(walkers_list)):
                all_id.append(walkers_list[i]["con"])
                all_id.append(walkers_list[i]["id"])
            all_actors = world.get_actors(all_id)

            # wait for a tick to ensure client receives the last transform of the walkers we have just created
            # if not args.sync or not synchronous_master:
            #     world.wait_for_tick()
            # else:
            world.tick()

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

            print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

            # example of how to use parameters
            traffic_manager.global_percentage_speed_difference(30.0)

        while True:
        

            actor_location = world.get_actor(vehicles_list[0]).get_location()
            actor_transform = world.get_actor(vehicles_list[0]).get_transform()
            actor_yaw = actor_transform.rotation.yaw
            spectator.set_transform(carla.Transform(actor_location+carla.Location(  z=10, 
                                                                                    x= - 10*math.cos(math.radians(actor_yaw)), 
                                                                                    y= - 10*math.sin(math.radians(actor_yaw))),
                                                                                    carla.Rotation(pitch= -30 ,yaw=actor_yaw)))
            world.wait_for_tick()

    # Wait for the user to end the script
    finally:

        # Clean up the actors
        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()

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

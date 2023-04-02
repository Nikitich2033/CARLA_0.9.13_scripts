#create a CARLA file to spawn 5 vehicles in the rain
import carla
import random
import time

actor_list = []
try:
    # connect to the server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # get the world
    world = client.get_world()

    # get the blueprint library
    bp_lib = world.get_blueprint_library()

    # get the list of vehicles
    vehicles = bp_lib.filter("vehicle.*")

    # get the list of spawn points
    spawn_points = world.get_map().get_spawn_points()

    # spawn 5 vehicles
    for n in range(5):
        # get a random vehicle
        vehicle_bp = random.choice(vehicles)
        # get a random spawn point
        spawn_point = random.choice(spawn_points)
        # spawn the vehicle
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        # add the vehicle to the list of actors
        actor_list.append(vehicle)

    # get the weather preset
    weather = carla.WeatherParameters(
        cloudiness=90.0,
        precipitation=60.0,
        precipitation_deposits=100.0,
        wind_intensity=0.0,
        sun_azimuth_angle=0.0,
        sun_altitude_angle=90.0
    )

    # set the weather
    world.set_weather(weather)

    # wait for 20 seconds
    time.sleep(20)

finally:
    # destroy all actors
    print('destroying actors')
    client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
    print('done.')
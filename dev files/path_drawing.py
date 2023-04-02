import carla
import random 
import math
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.basic_agent import BasicAgent

client = carla.Client("localhost", 2000)
client.set_timeout(10)
world = client.load_world('Town07')
traffic_manager = client.get_trafficmanager(8000)
traffic_manager.set_global_distance_to_leading_vehicle(1.0)
traffic_manager.set_synchronous_mode(True)


amap = world.get_map()
sampling_resolution = 2


def spawn_random_pedestrians_and_cars(world, route):
    # Get the blueprint library
    blueprint_library = world.get_blueprint_library()

    # Get the pedestrian and car blueprints
    pedestrian_blueprints = blueprint_library.filter("walker.pedestrian.*")
    car_blueprints = blueprint_library.filter("vehicle.*")

    # Set the number of pedestrians and cars to spawn
    num_pedestrians = 0
    num_cars = 5

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
            vehicle_actor.set_autopilot(True)


dao = GlobalRoutePlannerDAO(amap, sampling_resolution)
grp = GlobalRoutePlanner(dao)
grp.setup()
spawn_points = world.get_map().get_spawn_points()

a = carla.Location(x=80, y=60, z=3)
b = carla.Location(x=1.85, y=-15, z=3)

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

# Spawn a vehicle and set it to drive to destination
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter("model3")[0]
vehicle_actor = world.spawn_actor(vehicle_bp, carla.Transform(a))


actor = vehicle_actor
actor_location = actor.get_location()
actor_transform = actor.get_transform()
actor_yaw = actor_transform.rotation.yaw
spectator = world.get_spectator()
spectator.set_transform(carla.Transform(actor_location+carla.Location(  z=  10, 
                                                                        x= -10*math.cos(math.radians(actor_yaw)), 
                                                                        y= -10*math.sin(math.radians(actor_yaw))),
                                                                        carla.Rotation(pitch= -30 ,yaw=actor_yaw)))

spawn_random_pedestrians_and_cars(world,w1)


agent = BasicAgent(vehicle_actor)

# Set the destination
location = b
destination = world.get_map().get_waypoint(location).transform

agent.set_global_plan(w1,stop_waypoint_creation=True, clean_queue=True)


# Follow the route
while True:
    if agent.done():
        break
    world.tick()
    control = agent.run_step()
    vehicle_actor.apply_control(control)
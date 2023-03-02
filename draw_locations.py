import carla
import time 
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.controller import VehiclePIDController
import math

# Create a new CARLA client
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Load the scenario
# 1 autoware
# 2 small town
# 3 big with elevation
# 4 massive highway
# 5 urban city with highway
world = client.load_world('Town07')



map = world.get_map()

#draws waypoints for a certain road 
def draw_waypoints(waypoints, road_id=None, life_time=50.0):
  spawned = False 
  for waypoint in waypoints:
    # if spawned == False: 
    #   bad_actor_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    #   bad_transform = carla.Transform(waypoint.transform.location + 20, carla.Rotation(yaw=0))
    #   bad_car = world.spawn_actor(bad_actor_bp, start_transform)
    #   spawned = True
    if(waypoint.road_id == road_id):
        world.debug.draw_string(waypoint.transform.location, str(road_id), draw_shadow=False,
                                   color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                   persistent_lines=True)

def draw_all_waypoints(waypoints, life_time=150.0):

  for waypoint in waypoints:
        world.debug.draw_string(waypoint.transform.location, str(waypoint.road_id) 
                                + " X: " + str(round(waypoint.transform.location.x,2)) 
                                + " Y: " + str(round(waypoint.transform.location.y,2)) 
                                  , draw_shadow=True,
                                   color=carla.Color(r=0, g=0, b=0), life_time=150,
                                   persistent_lines=False,
                                   )
def draw_all_spawnpoints():
  spawn_points = world.get_map().get_spawn_points()
  for spawn_point in spawn_points:
    world.debug.draw_point(spawn_point.location, size=0.2, color=carla.Color(0, 255, 0), life_time=0.0)

    
waypoints = client.get_world().get_map().generate_waypoints(distance=5.0)

draw_all_waypoints(waypoints)
draw_all_spawnpoints()





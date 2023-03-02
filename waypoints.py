import carla
import time 
# Create a new CARLA client
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Load the scenario
world = client.load_world('Town03')

# # Get the blueprint for a Cybertruck
cybertruck_bp = world.get_blueprint_library().find('vehicle.tesla.cybertruck')

# # Set the starting location and orientation for the Cybertruck
start_transform = carla.Transform(carla.Location(x=150, y=-100, z=8.5), carla.Rotation(yaw=0))

# # Spawn the Cybertruck at the specified location and orientation
cybertruck = world.spawn_actor(cybertruck_bp, start_transform)

# # # Get the waypoint at the current location of the Cybertruck
# # waypoint = map.get_location(cybertruck.get_location())

# # # Get the next waypoint in front of the Cybertruck
# # next_waypoint = waypoint.next()
map = world.get_map()


# spawn_points = world.get_map().get_spawn_points()

# # Nearest waypoint on the center of a Driving or Sidewalk lane.
# waypoint01 = map.get_waypoint(cybertruck.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))

# waypoints = map.generate_waypoints(5)
# # waypoints = map.next_until_lane_end(5)
# for w in waypoints:
#     world.debug.draw_string(w.transform.location, str(w.transform.location.x) + " , " + str(w.transform.location.y) + " , " + str(w.transform.location.z), draw_shadow=False,
#                                        color=carla.Color(r=255, g=0, b=0), life_time=120.0,
#                                        persistent_lines=True)

# cybertruck.set_velocity(carla.Vector3D(x=50, y=0, z=0))
                                       
# #Nearest waypoint but specifying OpenDRIVE parameters. 
# waypoint02 = map.get_waypoint_xodr(road_id,lane_id,s)

# waypoint_list = map.generate_waypoints(2.0)

def draw_waypoints(waypoints, road_id=None, life_time=50.0):

  for waypoint in waypoints:

    if(waypoint.road_id == road_id):
        world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                   color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                   persistent_lines=True)



                              
waypoints = client.get_world().get_map().generate_waypoints(distance=1.0)
draw_waypoints(waypoints, road_id=24, life_time=100)

# time.sleep(8)  

from agents.navigation.basic_agent import BasicAgent
from agents.navigation.controller import VehiclePIDController


filtered_waypoints = []
for waypoint in waypoints:
    if(waypoint.road_id == 24):
      filtered_waypoints.append(waypoint)

spawn_point = filtered_waypoints[10].transform
spawn_point.location.z += 2

vehicle = client.get_world().spawn_actor(cybertruck_bp, spawn_point)
agent = BasicAgent(vehicle)

# Get the spectator camera
spectator = world.get_spectator()

# Get the position of an actor in the scene
actor = vehicle
actor_location = actor.get_location()

# Set the camera to look at the actor
spectator.set_transform(carla.Transform(actor_location+carla.Location(z=40), carla.Rotation(pitch=-90)))

target_waypoint = filtered_waypoints[50].transform
agent.set_destination([target_waypoint.location.x, 
                       target_waypoint.location.y, 
                       target_waypoint.location.z])
client.get_world().debug.draw_string(target_waypoint.location, 
                                        'O', draw_shadow=False,
                                        color=carla.Color(r=255, g=0, b=0), life_time=20,
                                        persistent_lines=True)



while True:
    if agent.done():
        print("The target has been reached, stopping the simulation")
        break

    vehicle.apply_control(agent.run_step())

# custom_controller = VehiclePIDController(cybertruck, args_lateral = {'K_P': 0, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0})
# ticks_to_track = 5
# for i in range(ticks_to_track):
#     control_signal = custom_controller.run_step(1, target_waypoint)
#     vehicle.apply_control(control_signal)
	
    
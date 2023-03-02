import carla
import time 
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.controller import VehiclePIDController
import math

# Create a new CARLA client
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)



# Load the scenario
world = client.load_world('Town03')

# Start the recorder
client.start_recorder("~/carla_0.9.10.1/PythonAPI/examples/additions/test.log", True)


# Get the blueprint for the pedestrian and set its attributes
blueprint_library = world.get_blueprint_library()
pedestrian_bp = blueprint_library.find('walker.pedestrian.0001')
pedestrian_bp.set_attribute('is_invincible', 'false')
pedestrian_bp.set_attribute('speed', '20.0')

pedestrian = world.spawn_actor(pedestrian_bp, carla.Transform(carla.Location(x=10, y=134, z=3.5),carla.Rotation(yaw=180)))
pedestrian_control = carla.WalkerControl()
pedestrian_control.speed = 20
pedestrian_control.direction = carla.Vector3D(x=-1.0, y=0.0, z=0.0)
pedestrian.apply_control(pedestrian_control)


# # Get the blueprint for a Cybertruck
cybertruck_bp = world.get_blueprint_library().find('vehicle.tesla.cybertruck')


bad_actor_bp = world.get_blueprint_library().find('vehicle.audi.etron')
bad_transform = carla.Transform(carla.Location(x= 13, y=130, z=3.5), carla.Rotation(yaw=180))
bad_car = world.spawn_actor(bad_actor_bp, bad_transform)




# vehicle_physics_control = cybertruck.get_physics_control()
# front_left_wheel = carla.WheelPhysicsControl(tire_friction=0.3)
# front_right_wheel = carla.WheelPhysicsControl(tire_friction=0.3)
# rear_left_wheel = carla.WheelPhysicsControl(tire_friction=0.3)
# rear_right_wheel = carla.WheelPhysicsControl(tire_friction=0.3)
# wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]

# vehicle_physics_control.wheels = wheels 
# cybertruck.apply_physics_control(vehicle_physics_control)


map = world.get_map()

#draws waypoints for a certain road 
def draw_waypoints(waypoints, road_id=None, life_time=50.0):
  spawned = False 
  for waypoint in waypoints:
    
    if(waypoint.road_id == road_id):
        world.debug.draw_string(waypoint.transform.location, str(road_id), draw_shadow=False,
                                   color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                   persistent_lines=True)

def draw_all_waypoints(waypoints, life_time=50.0):

  for waypoint in waypoints:
        world.debug.draw_string(waypoint.transform.location, str(waypoint.road_id), draw_shadow=False,
                                   color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                   persistent_lines=True)

waypoints = client.get_world().get_map().generate_waypoints(distance=1.0)
# draw_all_waypoints(waypoints)
draw_waypoints(waypoints, road_id=24, life_time=100)
draw_waypoints(waypoints, road_id=30, life_time=100)
draw_waypoints(waypoints, road_id=110, life_time=100)



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

target_road_waypoints = []
for waypoint in waypoints:
    if(waypoint.road_id == 30):
      target_road_waypoints.append(waypoint)

target_waypoint = target_road_waypoints[len(target_road_waypoints)-1].transform

agent.set_destination([target_waypoint.location.x, 
                       target_waypoint.location.y, 
                       target_waypoint.location.z])

client.get_world().debug.draw_string(target_waypoint.location, 
                                        'O', draw_shadow=False,
                                        color=carla.Color(r=255, g=0, b=0), life_time=0,
                                        persistent_lines=True)




while True:
    actor_location = actor.get_location()
    actor_transform = actor.get_transform()
    actor_yaw = actor_transform.rotation.yaw
    spectator.set_transform(carla.Transform(actor_location+carla.Location(  z=10, 
                                                                            x= - 10*math.cos(math.radians(actor_yaw)), 
                                                                            y= - 10*math.sin(math.radians(actor_yaw))),
                                                                            carla.Rotation(pitch= -30 ,yaw=actor_yaw)))
    
    distance = actor_location.distance(bad_car.get_location())
    print(f"The distance between actor 1 and actor 2 is {distance:.2f} meters.")
    if distance < 35:
      # custom_controller = VehiclePIDController(bad_car,throttle=1, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0}, args_longitudinal = {'K_P': 50, 'K_D': 40.0, 'K_I': 40.0,})
      
      # filtered_waypoints = []
      # for waypoint in waypoints:
      #     if(waypoint.road_id == 24):
      #       filtered_waypoints.append(waypoint)

      # target_waypoint = filtered_waypoints[20]

      # client.get_world().debug.draw_string(target_waypoint.transform.location, 'O', draw_shadow=False,
      #                           color=carla.Color(r=255, g=0, b=0), life_time=20,
      #                           persistent_lines=True)

      # ticks_to_track = 20
      # for i in range(ticks_to_track):

      # control_signal = custom_controller.run_step(20, target_waypoint)

      # bad_car.set_velocity(50)
      # bad_car.apply_control(control_signal)

      bad_control = carla.VehicleControl()
      throttle = 1.0
      control = carla.VehicleControl(throttle=throttle)
      bad_car.apply_control(control)

    if agent.done():
      print("The target has been reached, stopping the simulation")
      # actors = world.get_actors()

      # # Destroy all actors in the world and count the number of destroyed actors
      # num_destroyed = 0
      # for actor in actors:
      #     actor.destroy()
      #     num_destroyed += 1

      # # Print the number of destroyed actors
      # print(f"Destroyed {num_destroyed} actors.")
      client.stop_recorder()
      
      break

    vehicle.apply_control(agent.run_step())
   

	
    
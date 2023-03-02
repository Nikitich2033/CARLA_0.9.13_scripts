import carla

# Create a new CARLA client
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# print(client.show_recorder_file_info("test.log",True))

client.replay_file("test.log", 0, 0, 0)

# write a program that would query a user about how many scenarios for CARLA 
# they want to create, what weather do they want with options like Sunny, Rain, Thunderstorm etc. 
# Also ask if they want any pedestrians crossing the road. Ask if they want the scenario to be at an intersection and 
# how many cars do they want around 
# the actor car. Save the user input as
#  an object in a json file. Write this in python.

import json

# Query the user for the number of scenarios
num_scenarios = int(input("How many scenarios do you want to create for CARLA? "))

# Query the user for the weather conditions
print("What weather do you want? ")
print("1. Sunny")
print("2. Rain")
print("3. Thunderstorm")
weather_choice = int(input())
if weather_choice == 1:
    weather = "Sunny"
elif weather_choice == 2:
    weather = "Rain"
elif weather_choice == 3:
    weather = "Thunderstorm"
else:
    weather = "Invalid choice"

# Query the user for the presence of pedestrians
print("Do you want pedestrians crossing the road? ")
print("1. Yes")
print("2. No")
pedestrian_choice = int(input())
if pedestrian_choice == 1:
    pedestrians = "Yes"
elif pedestrian_choice == 2:
    pedestrians = "No"
else:
    pedestrians = "Invalid choice"

# Query the user for the presence of an intersection
print("Do you want the scenario to be at an intersection? ")
print("1. Yes")
print("2. No")
intersection_choice = int(input())
if intersection_choice == 1:
    intersection = "Yes"
elif intersection_choice == 2:
    intersection = "No"
else:
    intersection = "Invalid choice"

# Query the user for the number of cars around the actor car
num_cars = int(input("How many cars do you want around the actor car? "))

# Create a dictionary to store the user's input
scenario_data = {
    "num_scenarios": num_scenarios,
    "weather": weather,
    "pedestrians": pedestrians,
    "intersection": intersection,
    "num_cars": num_cars
}

# Save the user's input as a JSON file
with open("user_input/scenario_data.json", "w") as file:
    json.dump(scenario_data, file)

print("Scenario data has been saved to scenario_data.json")

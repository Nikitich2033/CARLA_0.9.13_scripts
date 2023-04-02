import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
import json

# Load the scenarios from the JSON file
with open('step1/random_scenarios.json') as f:
    scenarios = json.load(f)

# Define the possible values for each variable
weather = ['Sunny', 'Rain', 'Thunderstorm']
vehicle = ['Small', 'Truck', 'Van']
traffic = ['Heavy', 'Light', 'Medium']
emergency = ['Yes', 'No']
timeOfDay = ['Day', 'Night', 'Dawn', 'Dusk']
location = ['Urban', 'Country', 'Downtown']
intersections = [0, 1, 2, 3, 4]
pedestrians = [True, False]
pedestrian_cross = [True, False]
route_length = [100, 150, 200, 250, 300, 350, 400]
num_scenarios = 4


# Create a dataframe with the count of each combination of variable values
counts = pd.DataFrame(0, index=weather, columns=location)
for scenario in scenarios:
    counts.loc[scenario['weather'], scenario['location']] += 1

# Plot the heatmap
sns.heatmap(counts, annot=True, cmap="YlGnBu")
plt.show()
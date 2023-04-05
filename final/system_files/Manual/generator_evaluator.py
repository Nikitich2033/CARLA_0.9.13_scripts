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


# Convert JSON data to a DataFrame
scenario_df = pd.DataFrame(scenarios)

# Create a 2x2 grid for the plots
fig, axs = plt.subplots(2, 2, figsize=(10, 10))

# Increase font size for better visibility
plt.rcParams.update({'font.size': 14})

# Plot the bell curve distribution graph
sns.histplot(scenario_df["total_difficulty_rating"], kde=True, bins=20, ax=axs[0, 0])
axs[0, 0].set_title("Distribution of Difficulty Rating")
axs[0, 0].set_xlabel("Total Difficulty Rating")
axs[0, 0].set_ylabel("Frequency")

# Create Weather vs Traffic heatmap
weather_traffic_counts = pd.crosstab(scenario_df["weather"], scenario_df["traffic"])
sns.heatmap(weather_traffic_counts, annot=True, cmap="YlGnBu", ax=axs[0, 1])
axs[0, 1].set_title("Weather vs Traffic")

# Create Weather vs Location heatmap
weather_location_counts = pd.crosstab(scenario_df["weather"], scenario_df["location"])
sns.heatmap(weather_location_counts, annot=True, cmap="YlGnBu", ax=axs[1, 0])
axs[1, 0].set_title("Weather vs Location")

# Create Traffic vs Location heatmap
traffic_location_counts = pd.crosstab(scenario_df["traffic"], scenario_df["location"])
sns.heatmap(traffic_location_counts, annot=True, cmap="YlGnBu", ax=axs[1, 1])
axs[1, 1].set_title("Traffic vs Location")

# Adjust the spacing between subplots
plt.subplots_adjust(hspace=0.4, wspace=0.4)

# Add a title to the top of the generated image
plt.suptitle("Scenario Analysis Heatmaps", fontsize=24, y=0.95)

# Save the grid of plots as a PNG file
plt.savefig("step2/scenario_analysis_heatmaps.png", dpi=300, bbox_inches='tight')
plt.close()
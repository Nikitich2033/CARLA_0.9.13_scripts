import json
import numpy as np
from keras.models import Sequential
from keras.layers import Dense
from sklearn.preprocessing import OneHotEncoder

# Load the data from the JSON file
with open('user_input/scenarios.json') as f:
    data = json.load(f)

# Extract the features and labels from the data
X = []
y = []
for example in data:
    X.append([
        example['weather'],
        example['vehicle'],
        example['traffic'],
        example['timeOfDay'],
        example['location'],
        example['intersections'],
        example['route_length'],

    ])
    y.append(example['total_difficulty_rating'])

# One-hot encode the features
# Create a list of indices corresponding to the categorical columns in X
cat_cols = [0, 1, 2, 3, 4, 5]

# Initialize the one-hot encoder
encoder = OneHotEncoder(categories='auto', sparse=False)

# Fit the encoder on X
encoder.fit(X)

# Transform X using the encoder
X_encoded = encoder.transform(X)

# Convert the data to NumPy arrays
X = X_encoded 
y = np.array(y)

# Define the neural network model
model = Sequential()
model.add(Dense(16, input_shape=(30,), activation='relu'))
model.add(Dense(8, activation='relu'))
model.add(Dense(4, activation='relu'))
model.add(Dense(2, activation='linear'))

# Compile the model
model.compile(loss='mean_squared_error', optimizer='adam')

# Train the model
model.fit(X, y, epochs=100, batch_size=32)

# Define a function to generate a new scenario object based on the predicted difficulty rating
def generate_scenario(model, encoder, weather, vehicle, traffic, timeOfDay, location, intersections, route_length):
    # Create a new scenario object with the given features
    scenario = {
        'weather': weather,
        'vehicle': vehicle,
        'traffic': traffic,
        'timeOfDay': timeOfDay,
        'location': location,
        'intersections': intersections,
        'route_length': route_length
    }

    # One-hot encode the features
    features = [[weather, vehicle, traffic, timeOfDay, location, intersections, route_length]]
    features_encoded = encoder.transform(features)

    # Predict the difficulty rating using the model
    predicted_rating = model.predict(features_encoded)[0][0]

    # Add the predicted difficulty rating to the scenario object
    scenario['total_difficulty_rating'] = predicted_rating

    return scenario

# Generate a new scenario object with some example features
new_scenario = generate_scenario(model, encoder, 'Sunny', 'Small', 'Medium', 'Night', 'Urban', 3, 300)
print(new_scenario)
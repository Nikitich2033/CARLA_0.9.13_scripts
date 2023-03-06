import json
import numpy as np
from keras.models import Sequential
from keras.layers import Dense
from sklearn.model_selection import train_test_split

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

# Load the data from the JSON file
with open('user_input/scenarios.json') as f:
    data = json.load(f)


# One-hot encode the features
from sklearn.preprocessing import OneHotEncoder

# Create a list of indices corresponding to the categorical columns in X
cat_cols = [0, 1, 2, 3, 4, 5]

# Initialize the one-hot encoder
encoder = OneHotEncoder(categories='auto', sparse=False)

# Fit the encoder on X
encoder.fit(X)

# Transform X using the encoder
X_encoded = encoder.transform(X)

# Print the shape of the encoded X
print(X_encoded.shape)

# Convert the data to NumPy arrays
X = X_encoded 
y = np.array(y)

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Define the neural network model
model = Sequential()
model.add(Dense(16, input_shape=(30,), activation='relu'))
model.add(Dense(8, activation='relu'))
model.add(Dense(4, activation='relu'))
model.add(Dense(2, activation='linear'))

# Compile the model
model.compile(loss='mean_squared_error', optimizer='adam')

# Train the model
model.fit(X_train, y_train, epochs=100, batch_size=32, validation_data=(X_test, y_test))

# Evaluate the model
loss = model.evaluate(X_test, y_test)
print('Test loss:', loss)

# Make predictions with the model
predictions = model.predict(X_test)
# predictions = np.nan_to_num(predictions, nan=0)



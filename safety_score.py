import numpy as np

def calculate_safety_score(theta, t, alpha, beta, sigma, eta):
    if t < theta:
        return sigma * (alpha * (theta**2 - t**2) + beta * (theta - t))
    else:
        return eta * (alpha * (theta**2 - t**2) + beta * (theta - t))

def calculate_path_safety_score(perfect_path, driven_path, alpha, beta, sigma, eta):
    safety_score = 0
    for i in range(len(perfect_path)):
        theta = np.sqrt(perfect_path[i][0]**2 + perfect_path[i][1]**2)
        t = np.sqrt((driven_path[i][0] - perfect_path[i][0])**2 + (driven_path[i][1] - perfect_path[i][1])**2)
        safety_score += calculate_safety_score(theta, t, alpha, beta, sigma, eta)
    return safety_score

def calculate_path_safety_score_AB(perfect_path, driven_path, alpha, beta, sigma, eta, velocity, acceleration):
    safety_score = 0
    for i in range(len(perfect_path)):
        theta = np.sqrt(perfect_path[i][0]**2 + perfect_path[i][1]**2)
        t = np.sqrt((driven_path[i][0] - perfect_path[i][0])**2 + (driven_path[i][1] - perfect_path[i][1])**2)
        alpha_i = alpha*(1+acceleration)
        beta_i = beta*(1+velocity)
        safety_score += calculate_safety_score(theta, t, alpha_i, beta_i, sigma, eta)
    return safety_score

# Example usage:
perfect_path = [(0,0), (1,1), (2,2), (3,3)]
driven_path = [(0,0.5), (1,1), (2,2), (3,3)]
alpha = 1 # alpha,beta - Indicating the velocity and acceleration of an AV and surrounding vehicles 
beta = 1
sigma = 1 # Reward or penalty on the level of safety, when $t$ is lower or higher than $\theta$, respectively
eta = 1
velocity = 30 # 30 km/h  velocity of vehicles around? 
acceleration = 0.5  # 0.5 m/s^2 acceleration of vehicles around?
score = calculate_path_safety_score_AB(perfect_path, driven_path, alpha, beta, sigma, eta, velocity, acceleration)
print(score)







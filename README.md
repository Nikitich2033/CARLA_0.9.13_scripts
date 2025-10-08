# Automated CARLA Driving Scenario Testing System

## Project Context

This automated testing system was developed as part of a final year project focused on autonomous vehicle safety evaluation using the CARLA simulator (version 0.9.13). The system provides a complete pipeline for generating, executing, and evaluating driving scenarios to assess autonomous driving agent performance.

**Author:** Nikitich2033  
**GitHub Profile:** [https://github.com/Nikitich2033](https://github.com/Nikitich2033)  
**Project Repository:** [CARLA_0.9.13_scripts](https://github.com/Nikitich2033/CARLA_0.9.13_scripts)

## Overview

The system implements a three-step automated pipeline for comprehensive testing of autonomous driving behaviors:

1. **Step 1: Scenario Generation** - Randomly generate diverse driving scenarios with varying parameters
2. **Step 2: Scenario Execution** - Automatically execute scenarios in CARLA simulator and collect telemetry data
3. **Step 3: Performance Evaluation** - Analyze driving performance and calculate safety scores

## System Architecture

### Pipeline Flow

```
basic_generator.py → auto_scenario_runner.py → safety_score_evaluator.py
     (step1)              (step2)                    (step3)
```

## Scripts Description

### Core Pipeline Scripts

#### 1. `basic_generator.py`
**Purpose:** Generates random driving scenarios with configurable parameters.

**Features:**
- Creates diverse scenario configurations with varying:
  - Weather conditions (Sunny, Rain, Thunderstorm)
  - Vehicle types (Small, Truck, Van)
  - Traffic density (Light, Medium, Heavy)
  - Time of day (Day, Night, Dawn, Dusk)
  - Locations (Urban, Country, Downtown)
  - Number of intersections (0-4)
  - Pedestrian presence and crossings
  - Route lengths (100-400 meters)
- Calculates difficulty ratings for each scenario
- Generates route waypoints using CARLA's GlobalRoutePlanner
- Outputs: `step1/random_scenarios.json` and `step1/random_route_{n}.json`

#### 2. `auto_scenario_runner.py`
**Purpose:** Executes generated scenarios in CARLA simulator with autonomous agent.

**Features:**
- Loads scenario configurations from step1
- Spawns vehicles, pedestrians, and emergency vehicles based on scenario parameters
- Sets weather and time-of-day conditions
- Uses BasicAgent for autonomous driving
- Implements collision and lane invasion detection sensors
- Records comprehensive telemetry data:
  - Vehicle location, velocity, acceleration
  - Throttle, brake, and steering inputs
  - Collision events
  - Lane crossing violations
  - Game time stamps
- Outputs: `step2/auto_scenario_{n}.json`

#### 3. `safety_score_evaluator.py`
**Purpose:** Analyzes driving performance and calculates safety scores.

**Features:**
- Compares driven path to ideal trajectory using Dynamic Time Warping (DTW)
- Evaluates multiple safety metrics:
  - Path deviation from ideal route
  - Number of collisions
  - Lane violations (solid and double-solid line crossings)
  - Average velocity and acceleration
  - Path length efficiency
- Calculates normalized safety scores (0-100)
- Generates detailed visualization graphs for each scenario
- Creates safety score distribution graphs
- Outputs: `step3/rated_scenarios.json` and visualization PNGs

### Enhanced and Alternative Scripts

#### 4. `enhanced_generator.py`
**Purpose:** Machine learning-enhanced scenario generator.

**Features:**
- Uses MLPRegressor (Multi-Layer Perceptron) to learn from rated scenarios
- Generates scenarios targeting specific difficulty ranges
- Predicts scenario outcomes based on previous results
- Enables more controlled testing of edge cases

#### 5. `filtered_auto_scenario_runner.py`
**Purpose:** Filtered version of the scenario runner with additional constraints.

**Features:**
- Similar to `auto_scenario_runner.py` with additional filtering logic
- Can skip or modify certain scenario parameters
- Useful for focused testing on specific scenario types

#### 6. `filtered_safety_score_evaluator.py`
**Purpose:** Filtered version of the safety evaluator.

**Features:**
- Evaluates specific subsets of scenarios
- Custom filtering criteria for analysis
- Targeted performance assessment

### Utility Scripts

#### 7. `generator_evaluator.py`
**Purpose:** Visualizes and analyzes the distribution of generated scenarios.

**Features:**
- Creates heatmaps showing:
  - Weather vs Traffic distributions
  - Weather vs Location distributions
  - Traffic vs Location distributions
- Plots difficulty rating distributions
- Validates scenario generator randomness and coverage
- Outputs: `step2/scenario_analysis_heatmaps.png`

## Prerequisites

### Software Requirements
- **CARLA Simulator** version 0.9.13
- **Python** 3.7 or higher
- **CARLA Python API** (included with CARLA)

### Python Dependencies
```bash
pip install numpy
pip install scipy
pip install fastdtw
pip install matplotlib
pip install seaborn
pip install pandas
pip install scikit-learn
```

### CARLA Setup
1. Download and extract CARLA 0.9.13
2. Add CARLA Python API to your Python path:
   ```bash
   export PYTHONPATH=$PYTHONPATH:/path/to/CARLA_0.9.13/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg
   export PYTHONPATH=$PYTHONPATH:/path/to/CARLA_0.9.13/PythonAPI/carla/agents
   ```

## Usage Instructions

### Directory Structure
Before running the scripts, create the required output directories:
```bash
mkdir -p step1 step2 step3
```

### Step 1: Generate Scenarios
```bash
python basic_generator.py
```
- Modify `num_scenarios` variable in the script to set desired number of scenarios (default: 80)
- Outputs scenario configurations to `step1/` directory

**Optional:** Visualize scenario distributions:
```bash
python generator_evaluator.py
```

### Step 2: Execute Scenarios
1. Start CARLA simulator:
   ```bash
   ./CarlaUE4.sh
   ```

2. In a separate terminal, run the scenario executor:
   ```bash
   python auto_scenario_runner.py
   ```
- The script will automatically resume from the last executed scenario if interrupted
- Press Ctrl+C to stop execution
- Outputs telemetry data to `step2/` directory

### Step 3: Evaluate Performance
```bash
python safety_score_evaluator.py
```
- Analyzes all scenarios in `step2/` directory
- Generates individual scenario reports and visualizations
- Creates overall safety score distribution graph
- Outputs results to `step3/` directory

### Using Enhanced Generator
For machine learning-enhanced scenario generation:
```bash
python enhanced_generator.py
```
- Requires existing rated scenarios in `step3/rated_scenarios.json`
- Trains on previous results to generate targeted scenarios

## Output Files

### Step 1 Outputs
- `step1/random_scenarios.json` - Scenario configuration database
- `step1/random_route_{n}.json` - Individual route waypoints for each scenario

### Step 2 Outputs
- `step2/auto_scenario_{n}.json` - Telemetry data for each executed scenario
- `step2/scenario_analysis_heatmaps.png` - Scenario distribution visualizations (from generator_evaluator.py)

### Step 3 Outputs
- `step3/rated_scenarios.json` - Evaluated scenarios with safety scores
- `step3/scenario_{n}_results.png` - Individual scenario performance graphs
- `step3/calculated_safety_score_dist.png` - Overall safety score distribution

## Safety Score Calculation

The safety score is a normalized metric (0-100) based on multiple factors:

- **Path Accuracy** - DTW distance from ideal trajectory
- **Collision Avoidance** - Number of unique collisions (heavily penalized)
- **Lane Discipline** - Solid and double-solid lane crossing violations
- **Driving Smoothness** - Average velocity and acceleration metrics
- **Route Efficiency** - Driven path length vs. ideal trajectory length

Higher scores indicate safer, more controlled driving behavior.

## Key Features

- **Fully Automated Pipeline** - Minimal manual intervention required
- **Resumable Execution** - Can stop and restart at any point
- **Comprehensive Metrics** - Multiple safety and performance indicators
- **Rich Visualizations** - Detailed graphs for analysis
- **Scalable Testing** - Generate and test hundreds of scenarios
- **Machine Learning Ready** - Enhanced generator learns from results

## Use Cases

- Autonomous driving algorithm validation
- Safety assessment across diverse conditions
- Edge case discovery and testing
- Performance benchmarking
- Algorithm comparison studies

## Troubleshooting

### CARLA Connection Issues
- Ensure CARLA simulator is running before executing scenarios
- Default connection: `localhost:2000`
- Increase timeout if needed in script: `client.set_timeout(20.0)`

### Python Path Issues
- Verify CARLA Python API is in PYTHONPATH
- Check Python version compatibility (3.7+)

### Directory Errors
- Ensure `step1/`, `step2/`, `step3/` directories exist
- Check write permissions

## Project Attribution

This work is part of a comprehensive study on autonomous vehicle safety evaluation. The system demonstrates practical application of:
- CARLA autonomous driving simulator
- Scenario-based testing methodologies
- Performance metric design and implementation
- Data-driven safety analysis

For more information about the author's work and projects, visit: [https://github.com/Nikitich2033](https://github.com/Nikitich2033)

## License

Please refer to the main repository for licensing information.

## Contact

For questions or collaboration opportunities, please reach out through GitHub: [@Nikitich2033](https://github.com/Nikitich2033)

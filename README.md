# Multi-Robot Collision Avoidance and Path Planning

## Overview
This repository contains a purely Python-based simulation for a decoupled, multi-agent pathfinding (MAPF) system. It is engineered to navigate a fleet of autonomous robots through a constrained grid environment while strictly managing physical engineering realities, specifically kinematic constraints and dynamic battery discharge rates.

This project utilizes **Classical AI** techniques exclusively. It avoids modern machine learning or reinforcement learning, relying entirely on a deterministic, highly interpretable **Prioritized Space-Time A* Search** algorithm to ensure optimal and collision-free routing.

## Key Features
* **Prioritized Space-Time A* Search:** Expands the traditional 2D search space into 4D ($x, y, \theta, t$) to handle dynamic obstacle avoidance and time-based reservations seamlessly.
* **Physical Battery Modeling:** Simulates realistic power consumption with asymmetric energy drains for idling, moving forward, and turning in place, alongside a non-linear Open Circuit Voltage (OCV) penalty as the State of Charge (SOC) drops.
* **Decoupled Multi-Agent Planning:** Uses space-time reservation tables (vertex and edge tracking) to coordinate multiple robots without a centralized controller, assigning priority based on task difficulty.
* **Dependency-Free Visualization:** Renders the simulation, including robot kinematics and live battery telemetry, entirely within Python using `matplotlib.animation`. This eliminates the need for external engineering software for visualization.

## System Architecture
1. **Physics and Environment Module:** Defines the `GridEnvironment` (navigable space and static obstacles) and the `BatteryModel` (power consumption logic and action costs).
2. **AI Solver Module:** Contains the `RCNode` state representation and the core `PrioritizedSpaceTimeAStar` search algorithm.
3. **Coordination Module:** The `PrioritizedPlanner` sorts agents by path difficulty and manages the shared dynamic reservation tables to prevent spatial and temporal collisions.
4. **Visualization Module:** The `Visualizer` dynamically tracks and renders the environment, robot movements, and SOC percentages step-by-step.

## Prerequisites
The simulation is designed to run purely in Python (ideal for Kaggle or Google Colab environments) and requires the following standard libraries:
* `Python 3.x`
* `numpy`
* `matplotlib`
* `IPython` (for HTML animation rendering in notebooks)

## Installation & Usage
1. Clone the repository to your local machine or workspace:
   ```bash
   git clone [https://github.com/Salil8/multi-robot-path-planning.git](https://github.com/yourusername/multi-robot-path-planning.git)

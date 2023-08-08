# Coding Exercise Solution
This repo contains the solution for the dexory coding exercise.

# Problem Statment: 
Create a Custom Controller plugin, which generates Control Commands (Linear and Angular Velocity) to track the path generated from Nav2_planner.

# Solution:
- We can use any path tracking algorithm like Pure Pursuit, Stanley, or MPC to generate control commands given reference waypoints. We can also use LQR, and iLQR algorithms if we want to consider vehicle dynamics to generate control commands.

- In this Assignment, we will be using Pure Pursuit and adopting it to 2D Turtlebot, and adding more features from Regulated Pure Pursuit to improve the performance.



## Before Implementing the algorithm:
- Creating the custom plugin


## Intro to Pure Pursuit:
- The Pure Pursuit Algorithm is a path-tracking algorithm that geometrically calculates curvature to generate control commands to track a given reference path.
- It chooses a goal pose at some Look-ahead distance and generates control commands.

  ![](https://github.com/SaiSugunSegu/dex_sugun_ws/assets/50354583/d8d8a58e-fc94-4c2f-888e-7e9fe2e3913c)

rather than calculating the steering angle to reach the goal point using a kinematic bicycle model, we can adapt the same to Turtlebot.  

## Adaptation of Pure Pursuit to 2D Turtle bot:
In this assignment, we will be transforming the global path into a local robot frame, here is the math behind Geometrical Calculating Curvature.

  ![](https://github.com/SaiSugunSegu/dex_sugun_ws/assets/50354583/22de34b4-3ac9-4f5e-870a-5e6d7df7c1d2)

So, we will calculate this curvature to generate Angular velocity (assuming Linear velocity is coming from some Longitudinal Planner or controller.)

### Curvature (K) = 1/R = (2*y)/ (Ld)2.

- Ld (Lookahead Distance) = Kd * V.
- Kd - Lookahead gain
- V - Current Velocity
  
## Methodology:
### Step 1: Transform Global Waypoints into Local Robot frame
### Step 2: Finding Lookahead Point
### Step 3: Find Control Commands to reach the Lookahead point

_Apply Control Commands to the robot and repeat till it reaches the final destination._

Added features.

### Step 4: Rotate to Path & Rotate to Goal
### Step 5: Regulation on Linear Velocity
### Step 6: Regulation on Collision Check







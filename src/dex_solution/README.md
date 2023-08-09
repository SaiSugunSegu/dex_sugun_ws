# Coding Exercise Solution
This repo contains the solution for the dexory coding exercise.

# Problem Statment: 
Create a Custom Controller plugin, which generates Control Commands (Linear and Angular Velocity) to track the path generated from Nav2_planner.

# Solution:
- We can use any path tracking algorithm like Pure Pursuit, Stanley, or MPC to generate control commands given reference waypoints. We can also use LQR, and iLQR algorithms if we want to consider vehicle dynamics to generate control commands.

- In this Assignment, we will be using Pure Pursuit and adopting it to 2D Turtlebot, and adding more features from Regulated Pure Pursuit to improve the performance.

## Before Implementing the algorithm:
- Creating the custom plugin using [tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html). Follows steps
  - Create Plugin with neccesary virtual functions of base class, so that these methods can be called at runtime by the controller server.
  - Export Plugin using pluginlib `PLUGINLIB_EXPORT_CLASS` and plugin description file `controller.xml`, with neccesary changes in Cmakelist.txt and package.xml
  - Create param.yaml file to feed the parameters to the plugin.


## Intro to Pure Pursuit:
- The Pure Pursuit Algorithm is a path-tracking algorithm that geometrically calculates curvature to generate control commands to track a given reference path.
- It chooses a goal pose at some Look-ahead distance and generates control commands.

<p align="center">
  <img src="https://github.com/SaiSugunSegu/dex_sugun_ws/assets/50354583/4712a25c-6fed-4cd1-b50c-f21ea9af4261" />
</p>

rather than calculating the steering angle to reach the goal point using a kinematic bicycle model, we can adapt the same to Turtlebot.  

## Adaptation of Pure Pursuit to 2D Turtle bot:
In this assignment, we will be transforming the global path into a local robot frame, here is the math behind Geometrical Calculating Curvature.

<p align="center">
  <img src="https://github.com/SaiSugunSegu/dex_sugun_ws/assets/50354583/c99528c2-3eed-46d2-9ade-1afcd865c385" />
</p>

So, we will calculate this curvature to generate Angular velocity (assuming Linear velocity is coming from some Longitudinal Planner or controller.)

### $Curvature (K) = {1 \over R} = {(2*y) \over (Ld)^2}$.

- Ld (Lookahead Distance) = Kd * V.
- Kd - Lookahead gain
- V - Current Velocity
  
## Methodology:
### Step 1: Transform Global Waypoints into Local Robot frame
- Find the Closest Pose Upper Bound to create a Local Cost Map.
- Transform the near part of the global plan into the robot's frame of reference.
- Path Pruning: Remove the portion of the global plan that we've already passed.

### Step 2: Finding Lookahead Point
- Use current velocity to calculate Lookahead Distance (ld).

  $Ld = V * Kd$
  - Here will use `lookahead_time_` in place of Kd
    
  $Ld = V * Td$

  - Ld - Lookahead Distance
  - Kd - Lookahead Gain
  - Td - Lookahead Time

- Find the First Point that is greater than the computed Lookahead distance by iterating the transformed plan. 

### Step 3: Find Control Commands to reach the Lookahead point
- Find the Curvature
  
  $K = {1 \over R} = {2y \over (ld)^2}$

  
- Assuming Linear Velocity is chosen.
- Angular Velocity = Curvature * Linear Velocity. 

__Note:__ _Apply Control Commands to the robot and repeat till it reaches the final destination_

## Added features.

### Step 4: Rotate to Path & Rotate to Goal
- Rotate to Path
  - If the slope of the path (to the goal pose) is greater than the threshold, we will rotate with maximum angular velocity.
- Rotate to Goal
  - If the distance to the goal (to the goal pose) is less than the threshold, we will rotate with maximum angular velocity.

### Step 5: Regulation on Linear Velocity
- Limit the linear velocity by curvature
  - $CurvatureVelocity = Velocity * (1 - {abs(Radius - MinRadius) \over MinRadius})$
  - `min_radius` can be termed as `regulated_linear_scaling_min_radius_`.
    
- Limit the linear velocity by distance to obstacles 
  -  $CostVelocity = {CostScalingGain * MinDistanceToObstacle \over CostScalingDist}$
  - `min_distance_to_obstacle` will be calculated from costmap wrt pose_cost

- Minimum of both heuristics are taken as modified linear velocity.

### Step 6: Regulation on Collision Check
- Calculate projection time using simple ${CostResolution (unit dist) \over Velocity}$
- Calculate possible future poses using a motion model
  - $x =  dt * (vel * yaw)$
  - $y =  dt * (vel * yaw)$
- Check the `footprint_cost` of those future potential poses to estimate collision.






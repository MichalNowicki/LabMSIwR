# 3d_planner

## Goal

The goal of this packge is to provide an ability to plan a 3D path in the 3D octomap.

## Topics

### Subscription

* ``poseTopic`` - the position of the robot in the map
* ``goalTopic`` - the position of the goal to plan the path
* ``octreeTopic`` - the octomap topic

### Publishing

* ``plannerStringTopic`` - some information about the current state of the planner
* ``plannedPathMarkersTopic`` - the visualization of the planned path in rviz

## Running and parameters

Running:

`` roslaunch planner_3d run_planner_3d.launch ``

You can set the following parameters:

* ``robotSizeXY`` (0.25) - The side of the square representing the robot size on the ground plane
* ``robotSizeZ`` (0.2) - The height of the robot
  
* ``planning_time_initial`` (15) - Time in seconds for the 1st phase of planning.
* ``planning_time_secondary`` (30) - Time in seconds for the 2nd phase of planning. Any correct solution stops further planning.
* ``planning_terminate_cond_check_interval`` (0.5) - Determines how often do we stop the planning to check for interruptions
* ``planning_objective`` (path_length) - The assumed type of planning: based on distance (path_length), based on distance to obstacles (clearance) or combination of both previous approaches (balanced_path_length_and_clearance).
* ``planning_balanced_obj_path_weight`` (10) - The weight between path distance and object clearance
* ``planning_objective_z_weight`` (5) - The coefficient for the assumed cost of moving up/down. Anythin above 1 means a preference to localization on the ground plane.

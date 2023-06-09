# Autonomous Transpalet Project

Welcome to the Autonomous Transpalet Project! This repository contains code and resources for an autonomous transpalet system implemented in the Mujoco simulator. The project aims to achieve autonomous transpalet actions by combining the Probabilistic Road Map (PRM) algorithm as a global planner and the Dynamic Window Approach (DWA) algorithm as a local planner.

This project incorporates pre-existing path planning algorithms from the Python Robotics repository (AtsushiSakai/PythonRobotics). Used pre-existing algorithms are named as dwa.py and prm.py. Also, the study named door action trial.py also directly overwritten the algorithm named dwa.py.

The Python Robotics repository is licensed under the MIT license and was created by Atsushi Sakai. Please refer to the [Python Robotics repository](https://github.com/AtsushiSakai/PythonRobotics) for more details.

## Table of Contents
- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [Project Explanation](#project-explanation)
- [About the Main Script](#about-the-main-script)

## Introduction

The Autonomous Transpalet Project focuses on developing an autonomous system that can control a transpalet to perform various tasks in a simulated environment. The system utilizes the power of the PRM algorithm for global path planning and the DWA algorithm for local trajectory generation. By combining these algorithms, the transpalet can autonomously navigate its environment, pick up palets, and move them to desired locations.

## Installation

To run the project, you need to set up the following dependencies:

- Python 3.x
- mujoco
- mujoco_viewer
- dm_control
- numpy
- math
- scipy
- simple_pid
- xml.etree.ElementTree

You can install the dependencies using pip:

```shell
pip install dm_control mujoco mujoco_viewer numpy scipy simple_pid xml.etree.ElementTree
```

Once the dependencies are installed, you can clone the repository:

```shell
git clone https://github.com/your_username/Auonomous-Transpalet-Project.git
```

## Usage
The script containing the actual work of the project is named "Main_script". In order to run this script, all necessary xml files and assets folder must be added to the main folder and the paths of these files must be properly assigned in the script. A site named "mid_point" has been added to the model.xml file, different from the first version, and the position and rotation information of the vehicle is obtained through this.

To run the project, navigate to the project directory and execute the main script:

```shell
python Main_script.py
```

The script will run the simulation using the specified parameters and display the results in the Mujoco viewer.

## Project Explanation
To bring the project to fruition, numerous libraries, diverse XML files for environment creation, various STL files for assigning appropriate meshes to objects, and pre-existing DWA and PRM algorithms have been employed.

The project utilizes the following techniques:

- Global Planner: Probabilistic Road Map (PRM)
- Local Planner: Dynamic Window Approach (DWA)
- Control Inputs: Angular velocity and speed, converted to relevant control inputs.

The mission of the autonomous transpalet is divided into several parts:

- Reaching the pallet: The transpalet navigates to the pallet using PRM and DWA.
- Pallet retrieval: The transpalet uses PID controllers considering lateral distance and angular difference to take the pallet.
- Pallet movement: The transpalet moves the pallet to the desired location using PRM and DWA.
- Pallet placement: The transpalet puts the pallet in the desired location with the help of PID controllers.
- Final position adjustment: The autonomous transpalet moves backwards and comes out from under the pallet, while the pallet remains in its final position.

By implementing this project, the aim is to showcase the capabilities of the autonomous transpalet system in performing various tasks in an automated manner within the Mujoco environment.

An algorithm was tried to be prepared for the vehicle that had difficulty in door passages, but it was successful in simplified simulations, but failed in the actual simulation. The algorithm and simplified simulation work can be accessed in the script named "door action trial".

The video showing the operation of the actual simulation has been added with the name "mujoco_simulation.webm", and the simple simulation video about the door passages has been added with the name "door action trial.webm".

Please refer to the Installation and Usage sections in the README file for detailed instructions on how to install and run the project.

## About the Main Script
The main script, encompassing numerous functions and algorithms and serving as the foundation of the project, is meticulously elucidated below. Each section is presented with clear headings, guiding readers through the sequence of steps in a methodical manner.

### Setting the Positions and the Rotations of the Both Transpalet and Pallet:
The code updates the positions and rotations of a transpalet and a pallet object in a 3D simulation environment. It achieves this by loading XML model files for both objects, parsing them into element trees, and modifying the attributes of specific elements to reflect the desired positions and rotations. Finally, the updated XML content is saved back to the original files, effectively applying the changes made to the object properties.

### palet_take_or_put_calculations:
The function calculates the control values for a vehicle's movement relative to a palet object. It takes parameters such as the palet index, position, rotation, midpoint position, and rotation angles. The function applies PID control to determine the steering angle and drive speed of the vehicle based on the goal vector, palet normal, and angle differences. The resulting control values are returned for controlling the vehicle's movement during palet handling operations.

### deciding_control_inputs:
The function calculates the control inputs for a vehicle based on the lateral and longitudinal drive velocities. It determines the drive speed control by considering the magnitude of the drive velocity and assigns a negative value if the longitudinal velocity is positive, or a positive value if the longitudinal velocity is negative. The steering angle control is calculated based on the steering drive angle, which is determined using the arctangent of the longitudinal and lateral velocities. The resulting steering angle control is scaled within the range of -1 to 1. The function returns the calculated drive speed control and steering angle control as the output for controlling the vehicle's movement.

### global_path_planning:
The function performs global path planning for a robot in an environment. It uses the Probabilistic Roadmap (PRM) planning algorithm to generate a path from the current position (x and y) to a goal position (gx and gy), avoiding obstacles (ox and oy). If necessary, the function attempts PRM planning multiple times, until a valid path is found. The resulting path is then modified, and the modified data is returned as the planned path for the robot's global navigation.

### find_dwa_goal:
The function calculates the goal point for the robot's dynamic window approach (DWA) path following. It determines the current position's s-coordinate using "xy_to_s", then adds the follow distance to obtain the DWA goal point in (x, y) coordinates using "s_to_xy". The resulting goal point is returned. The functions "s_to_xy" and "xy_to_s" are helper functions used to convert between s coordinates and (x, y) coordinates, retrieving the corresponding values from modified PRM data.

### modify_prm_road:
The function modifies a given PRM (Probabilistic Roadmap) road by adding intermediate points along straight line segments between consecutive points. It takes the PRM road data "road_prm" and a step size as parameters. The function first reverses the order of the "road_prm" data using "np.flip". It then iterates over the reversed "road_prm" data and performs linear interpolation between consecutive points using "lin_data". The interpolated road segments are added to the "modified_prm_data" array, excluding the last point of each segment. The cumulative s-coordinate data "s_prm_data" is calculated based on the Euclidean distances between consecutive points in the "modified_prm_data". The function returns "s_prm_data" and "modified_prm_data" as the modified PRM road data.

### remove_duplicates:
The function removes duplicate rows from a NumPy ndarray. It converts the ndarray into a set of tuples to eliminate duplicates and then converts it back into a NumPy ndarray called unique_arr that contains only the unique rows. The function returns unique_arr as the modified ndarray with duplicates removed.

### lin_data:
The function generates a linearly interpolated set of points between a given start and end point. It takes the start point, end point, and step size as parameters. The function first converts the start and end points into NumPy arrays. It then calculates the direction vector as the difference between the end and start points. The distance between the two points is determined using the Euclidean norm. The function generates a sequence of values using np.linspace based on the distance and step size, representing the distances along the line segment. These distances are then scaled and added to the start point to obtain the interpolated points. The resulting points are returned as an ndarray.

### check_integer_multp:
The function checks if two numbers, num1 and num2, are integer multiples of each other. It first calculates the value of num1 divided by num2 and rounds the result. The function then calculates the absolute difference between num1 and the rounded value multiplied by num2. If the error is less than or equal to a small threshold (1e-6), it returns True, indicating that the numbers are integer multiples. Otherwise, it returns False.

### model_angle_correction:
The function performs angle correction for a given angle, angle_model. It checks if the angle is greater than or equal to zero and subtracts π (pi) from it if true, or adds π if the angle is negative. The resulting corrected angle, angle_dwa, is then returned. This function is useful for adjusting angle values within a specific range or format, such as for compatibility with other calculations or conventions.

### Extracting Wall Information:
The provided code extracts information about the walls in a simulation environment. It iterates over each wall, determines the coordinates of its corners, and stores them in a list. Using the collected corner coordinates, the code generates a set of points along each side of the wall. These points are concatenated and stored as obstacle information. Duplicate points are removed, resulting in a final set of unique obstacle coordinates. The x and y coordinates of the obstacles are then separated into separate arrays for further use.

### Managing Dynamic Objects:
This code segment involves the management of dynamic objects in a simulation environment. It first defines a list, dynamic_object_actuators, which contains references to the actuators of the dynamic objects in the simulation. Similarly, a list called dynamic_object_positions is created to store references to the positions of the dynamic objects. Additionally, a variable dynamic_object_speed is set to a value of 0.2, representing the speed of the dynamic objects.
- update_dynamic_objects: This function takes a step_counter parameter and iterates through each dynamic object using a for loop. Within the loop, it modifies the ctrl attribute of each dynamic object's actuator based on a sine wave determined by the step_counter value. This manipulation alters the control input of the dynamic objects over time.
- update_dynamic_obs: This function receives two parameters, obs and dynamic_object_positions. It initializes a local variable ob to store the current state of the observations. Then, it iterates through each dynamic object's position and appends the 2D position (x, y) to ob using np.vstack(), effectively including the dynamic object positions in the set of observations. Finally, the modified ob array is returned.

### Control Algorithm for Reaching the Pallet:
The code segment represents a control algorithm responsible for reaching the pallet. It is structured as a conditional flow, where different actions are taken based on the current state.
- When the condition reach_palet is true, the algorithm executes a series of steps. In the first iteration (when first_action is true), a global path planning function, global_path_planning(), is called to determine a path from the current position (transpalet_pos) to the pallet's position (palet_pos). The resulting path is stored in s_prm_data and modified_prm_data.
- Next, a goal position is determined using the find_dwa_goal() function, which calculates a point on the path that is a certain distance (4.0) ahead of the current position (mid_point_pos). This goal position is used as a reference for subsequent motion planning.
- The algorithm then enters a loop where it continuously updates the control inputs (u) using the Dynamic Window Approach (DWA) control algorithm (dwa.dwa_control()). The DWA control function takes into account the current state (x), configuration of the dwa, the goal position, and obstacle information (ob). The resulting control inputs represent longitudinal (long_drive_vel) and lateral (lat_drive_vel) velocities.
- The function deciding_control_inputs() is then called to map the calculated velocities to control signals (drive_speed_actuator.ctrl and steering_angle_actuator.ctrl). These signals determine the desired drive speed and steering angle for the vehicle.
- The algorithm also keeps track of the distance to the main goal (dist_to_main_goal) and the progress along the path (s_dist_to_main_goal) using the xy_to_s() function. These values are used to trigger specific actions. For instance, when dist_to_main_goal is less than or equal to 5 and s_dist_to_main_goal is less than or equal to 7, it indicates that the vehicle is close to the pallet and the desired position for interaction. In such cases, the drive speed and steering angle signals are set to zero (drive_speed_actuator.ctrl = 0 and steering_angle_actuator.ctrl = 0), and the variable end_action is set to true to indicate the completion of the current action sequence.


### Control Algorithm for Picking Up the Pallet:
The provided code segment is part of a control algorithm for picking up the pallet. It operates within a conditional statement where different actions are executed based on the state.
- When the condition take_palet is satisfied, the algorithm performs a series of steps. In the initial iteration (when first_action is true), the code calculates the direction vector of the pallet (palet_direction) and its perpendicular normal vector (palet_normal) based on the pallet's rotation angle (palet_rot). It then defines two potential entry points (palet_enters) for approaching the pallet, located at a distance of 3 times the pallet direction away from the pallet's position (palet_pos).
- Next, the algorithm determines the closest entry point to the current position (mid_point_pos) by calculating the distances (palet_enter_dists) between each entry point and the current position. The index of the closest entry point is stored in palet_index, and the variable first_action is set to false.
- The control signals for the drive speed, steering angle, and fork height (drive_speed_actuator.ctrl, steering_angle_actuator.ctrl, fork_height_actuator.ctrl) are then calculated by calling the function palet_take_or_put_calculations(), which takes as inputs the pallet index, normal vector, pallet position, pallet rotation angle, current position, and current rotation angles.
- The code also tracks the distance to the main goal (dist_to_main_goal), which represents the remaining distance between the current position and the pallet position. If dist_to_main_goal is less than or equal to 0.3 and the condition take_palet is satisfied, it indicates that the vehicle is close to the pallet and ready for interaction. In this case, the drive speed and steering angle signals are set to zero (drive_speed_actuator.ctrl = 0 and steering_angle_actuator.ctrl = 0), and the fork height signal is set to its maximum value (fork_height_actuator.ctrl = 1.0). The variable end_action is then set to true to signal the completion of the current action sequence.
Control Algorithm for Moving the Pallet: The provided code segment is part of a control algorithm that handles the movement of the pallet. It is very similar to “Control Algorithm for Reaching the Pallet”.

### Control Algorithm for Putting Down the Pallet:
The provided code segment represents a control algorithm for putting down the pallet. It is very similar to “Control Algorithm for Picking Up the Pallet”.

### Control Strategy for Moving the Robot Back:
This code segment demonstrates the control strategy for moving the robot back to a specific position after completing a task. In the "move_back" state, the algorithm sets the control inputs to make the robot move backward at a constant speed of 0.1 and with a steering angle of 0, effectively driving in a straight line. It calculates the distance between the current robot position and the designated back point using the euclidean distance formula. If the distance is less than or equal to 0.1, indicating that the robot has reached the back point, the control inputs are set to zero to stop the robot's motion.

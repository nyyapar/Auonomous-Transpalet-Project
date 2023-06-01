# Autonomous Transpalet Project

Welcome to the Autonomous Transpalet Project! This repository contains code and resources for an autonomous transpalet system implemented in the Mujoco simulator. The project aims to achieve autonomous transpalet actions by combining the Probabilistic Road Map (PRM) algorithm as a global planner and the Dynamic Window Approach (DWA) algorithm as a local planner.

## Table of Contents
- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [Project Explanation](#project-explanation)

## Introduction

The Autonomous Transpalet Project focuses on developing an autonomous system that can control a transpalet to perform various tasks in a simulated environment. The system utilizes the power of the PRM algorithm for global path planning and the DWA algorithm for local trajectory generation. By combining these algorithms, the transpalet can autonomously navigate its environment, pick up palets, and move them to desired locations.

## Installation

To run the project, you need to set up the following dependencies:

- Python 3.x
- Mujoco
- dm_control
- numpy
- math
- scipy
- simple_pid

You can install the dependencies using pip:

```shell
pip install dm_control mujoco numpy scipy simple_pid
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

The project utilizes the following techniques:

    Global Planner: Probabilistic Road Map (PRM)
    Local Planner: Dynamic Window Approach (DWA)
    Control Inputs: Angular velocity and speed, converted to relevant control inputs.

The mission of the autonomous transpalet is divided into several parts:

    Reaching the pallet: The transpalet navigates to the pallet using PRM and DWA.
    Pallet retrieval: The transpalet uses PID controllers considering lateral distance and angular difference to take the pallet.
    Pallet movement: The transpalet moves the pallet to the desired location using PRM and DWA.
    Pallet placement: The transpalet puts the pallet in the desired location with the help of PID controllers.
    Final position adjustment: The autonomous transpalet moves backwards and comes out from under the pallet, while the pallet remains in its final position.

By implementing this project, the aim is to showcase the capabilities of the autonomous transpalet system in performing various tasks in an automated manner within the Mujoco environment.

An algorithm was tried to be prepared for the vehicle that had difficulty in door passages, but it was successful in simplified simulations, but failed in the actual simulation. The algorithm and simplified simulation work can be accessed in the script named "door action trial".

The video showing the operation of the actual simulation has been added with the name "mujoco_simulation.webm", and the simple simulation video about the door passages has been added with the name "door action trial.webm".

Please refer to the Installation and Usage sections in the README file for detailed instructions on how to install and run the project.

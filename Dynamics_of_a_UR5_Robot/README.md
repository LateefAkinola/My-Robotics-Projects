The UR5 is a popular 6-DOF industrial robot arm.  The robot has geared motors at each joint, but in this project, we ignore the effects of gearing, such as friction and the increased apparent inertia of the rotor.

The relevant kinematic and inertial parameters of the UR5 are given in the UR5_parameters.py file.

GOAL: To write code that simulates the motion of the UR5 for a specified amount of time (in seconds), from a specified initial configuration (at zero velocity), when zero torques are applied to the joints.  In other words, the robot simply falls in gravity. Gravity is g=9.81m/s^2 in the -Z direction, i.e., gravity acts downward. The motion should be simulated with at least 100 integration steps per second.  The program should calculate and record the robot joint angles at each step. This data should be saved as a .csv file, where each row has six numbers separated by commas. This .csv file is suitable for animation with the CoppeliaSim UR5 csv animation scene.

Two simulations were performed and the videos of each were recorded:

1.  The robot falling from the zero (home) configuration for 3 seconds.

2.  The robot falling from a configuration where all joints are at their zero position, except for joint 2, which is at −1 radian.  This simulation should last 5 seconds.

=================IMPORTANT INFOMATION ABOUT THE FILES====================

<> These python files contains the written functions utilized and they generate the csv files for simulation:
1. "Defined_Functions.py": Contains written functions to perform the forward dynamics and write matrix into csv 
2. "Task_Execution.py": This contains all parameters used and how the Defined Functions were executed to achieve the goal

<> 'Simulation1.csv' and 'Simulation2.csv' are generated using the written functions.

<> 'Simulation1.Avi' and 'Simulation2.Avi' are recorded videos of simulations in CoppeliaSim.
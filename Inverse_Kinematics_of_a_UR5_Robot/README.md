# Inverse kinematics of the UR5 robot
GOAL: To write a new function, IKinBodyIterates, and to test your new function for the UR5 robot.

This function prints out a report for each iteration of the Newton-Raphson process, for iterates 0 (the initial guess) to the final solution. Each iteration reports the iteration number i, the joint vector, the SE(3) end-effector configuration, the error twist V_b, and the angular and linear error magnitudes, ∥omega_b∥ and ∥v_b∥. For a four-joint robot, a typical iterate might look like:

Iteration 3:

joint vector:0.221, 0.375, 2.233, 1.414

SE(3) end−effector config:[[1.000, 0.000, 0.000, 3.275], [0.000, 1.000, 0.000, 4.162], [0.000, 0.000, 1.000, −5.732], [0, 0, 0, 1]]

error twist V_b: 0.232, 0.171, 0.211, 0.345, 1.367, −0.222

angular error magnitude ∣∣omega_b∣∣: 0.357

linear error magnitude ∣∣v_b∣∣: 1.427

=================IMPORTANT INFOMATION ABOUT THE FILES====================

<> These MATLAB files contains the written functions utilized: 
1. IKinBodyIterates.m: Modified IKinBody function to suit the goal stated.
2. PrintIterationValues.m: This is used in the 'IKinBodyIterates' function to print the iteration values.   

<> The "CoppeliaSim_Animation.Avi" is a recorded video animating the Newton-Raphson iterations. The video shows CoppeliaSim playing the "iterates.csv" file

<> The "log.txt" file shows the call of IKinBodyIterates with the initial guess, as well as all of the Newton-Raphson iterations until convergence.

<> The "iterates.csv" file is created by the IKinBodyIterates function when it created the "log.txt" file.

<> The "screenshot.png" showing the UR5 at the solution.  This was made with the UR5 interactive scene, and the screenshot clearly shows the UR5's end-effector configuration as well as the SE(3) configuration reported by the scene's interface, confirming that your code calculated a good solution.

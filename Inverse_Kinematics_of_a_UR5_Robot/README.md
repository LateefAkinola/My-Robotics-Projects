# Inverse kinematics of the UR5 robot
GOAL: To write a new function, IKinBodyIterates, and to test your new function for the UR5 robot.

This function prints out a report for each iteration of the Newton-Raphson process, for iterates 0 (the initial guess) to the final solution. Each iteration reports the iteration number i, the joint vector, the SE(3) end-effector configuration, the error twist V_b, and the angular and linear error magnitudes, ∥omega_b∥ and ∥v_b∥. For a four-joint robot, a typical iterate might look like:

Iteration 3:
joint vector:0.221, 0.375, 2.233, 1.414
SE(3) end−effector config:[[1.000, 0.000, 0.000, 3.275], [0.000, 1.000, 0.000, 4.162], [0.000, 0.000, 1.000, −5.732], [0, 0, 0, 1]]
error twist V_b: 0.232, 0.171, 0.211, 0.345, 1.367, −0.222
angular error magnitude ∣∣omega_b∣∣: 0.357
linear error magnitude ∣∣v_b∣∣: 1.427

==================================IMPORTANT==================================

<> The code folder (written in MATLAB) consists of two "function '.txt' " files
that contains the codes utilized: 
1. IKinBodyIterates.txt: Modified IKinBody function to suit the assignment 
			 instructions 
2. PrintIterationValues.txt: This is used in the 'IKinBodyIterates' function
			     to print the iteration values.   

<> The CoppeliaSim Animation.Avi is a recorded video of how the interation
converges. The file will play in almost every media application that supports
'.mp4' files like the VLC and Windows Media Player.

<> The 'log.txt', 'Screenshot.png', and 'Iterates.csv' files are in accordance
to the instructions
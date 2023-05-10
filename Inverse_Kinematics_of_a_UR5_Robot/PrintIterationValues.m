function PrintIterationValues(i,thetalist,Tsb,Vb,magVbw,magVbv)

% Takes i: The iteration number,
%       thetalist: An values of  joint angles that satisfy Tsb,
%       Tsb: The end-effector configuration Tsb resulting from the values
%            of thetalist,
%       Vb: The twist resulting from Tsb,
%       magVbw: The angular error magnitude,
%       magVbv: The linear error magnitude.
%
% Prints   Each iteration reports the iteration number i, the joint vector, 
%            the end-effector configuration, the error twist Vb, and the
%            angular and linear error magnitudes

% Example Inputs:
% 
% clear; clc;
% i = 1;
% thetalist = [1.5824; 2.9748; 3.1531];
% Tsb = [[-0.0001 1 0 -4.9745]; [1 0.0001 0 3.9423]; [0 0 -1 1.6847]; [0 0 0 1]];
% Vb = [ 0; 0; 0.0001; 0.0577; -0.0255; -0.0011];
% magVbw = 0.00009632679;
% magVbv = 0.06313002;
% PrintIterationValues(i,thetalist,Tsb,Vb,magVbw,magVbv)

% Output:
% Iteration 0: 
% Joint Vector:
%     1.5824    2.9748    3.1531
% 
% SE(3) end-effector config:
%    -0.0001    1.0000         0   -4.9745
%     1.0000    0.0001         0    3.9423
%          0         0   -1.0000    1.6847
%          0         0         0    1.0000
% 
% Error twist Vb:
%          0         0    0.0001    0.0577   -0.0255   -0.0011
% 
% Angular Error Magnitude: 9.632679e-05 
%  Linear Error Magnitude: 6.313002e-02 
 

%--------START-----------

fprintf('Iteration %d: \n', i); % prints the iteration number 
disp('Joint Vector:');        
disp(thetalist');               % prints the joint vector values as a row vector
disp('SE(3) end-effector config:'); 
disp(Tsb);                      % prints End-Effector config
disp('Error twist Vb:');
disp(Vb');                      % prints Twist as a row vector
fprintf('Angular Error Magnitude: %d \n ', magVbw);     % prints the Mag of Vbw
fprintf('Linear Error Magnitude: %d \n \n \n', magVbv); % prints the Mag of Vbv

end
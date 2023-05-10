function [thetalist, success] = IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)

% *** CHAPTER 6: INVERSE KINEMATICS ***
% Takes Blist: The joint screw axes in the end-effector frame when the
%              manipulator is at the home position, in the format of a 
%              matrix with the screw axes as the columns,
%       M: The home configuration of the end-effector,
%       T: The desired end-effector configuration Tsd,
%       thetalist0: An initial guess of joint angles that are close to 
%                   satisfying Tsd,
%       eomg: A small positive tolerance on the end-effector orientation
%             error. The returned joint angles must give an end-effector 
%             orientation error less than eomg,
%       ev: A small positive tolerance on the end-effector linear position 
%           error. The returned joint angles must give an end-effector
%           position error less than ev.
% Returns   Each iteration reports the iteration number i, the joint vector, 
%               the end-effector configuration, the error twist Vb, and the
%               angular and linear error magnitudes
%           thetalist: Joint angles that achieve T within the specified 
%                    tolerances,
%           success: A logical value where TRUE means that the function found
%                  a solution and FALSE means that it ran through the set 
%                  number of maximum iterations without finding a solution
%                  within the tolerances eomg and ev.
% Uses an iterative Newton-Raphson root-finding method.
% The maximum number of iterations before the algorithm is terminated has 
% been hardcoded in as a variable called maxiterations. It is set to 20 at 
% the start of the function, but can be changed if needed.  
% Example Inputs:
% 
% clear; clc;
% Blist = [[0; 0; -1; 2; 0; 0], [0; 0; 0; 0; 1; 0], [0; 0; 1; 0; 0; 0.1]];
% M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
% T = [[0, 1, 0, -5]; [1, 0, 0, 4]; [0, 0, -1, 1.6858]; [0, 0, 0, 1]];
% thetalist0 = [1.5824; 2.9748; 3.1531];
% eomg = 0.01;
% ev = 0.001;
% [thetalist, success] = IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
% 
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
%  
%  
% Iteration 1: 
% Joint Vector:
%     1.5707    2.9997    3.1415
% 
% SE(3) end-effector config:
%     0.0000    1.0000         0   -4.9997
%     1.0000   -0.0000         0    4.0003
%          0         0   -1.0000    1.6858
%          0         0         0    1.0000
% 
% Error twist Vb:
%    1.0e-03 *
% 
%          0         0   -0.0046   -0.2904   -0.3338    0.0461
% 
% Angular Error Magnitude: 4.608282e-06 
%  Linear Error Magnitude: 4.448730e-04 
%  
%  
% Joint Vector Matrix = 
%     1.5824    2.9748    3.1531
%     1.5707    2.9997    3.1415
% 
% 
% thetalist =
% 
%     1.5707
%     2.9997
%     3.1415
% 
% 
% success =
% 
%    1


%-----START-----------

thetalist = thetalist0;
i = 0;
maxiterations = 20;

Tsb = FKinBody(M, Blist, thetalist); % Returns end-effector configuration
Vb = se3ToVec(MatrixLog6(TransInv(Tsb) * T));   %   Returns Twist
magVbw = norm(Vb(1: 3));    %  Returns angular error magnitude
magVbv = norm(Vb(4: 6));    %  Returns linear error magnitude
err = magVbw > eomg || magVbv > ev;

jointVecMatrix = zeros(1, numel(thetalist));
jointVecMatrix((i+1),:) = thetalist';

PrintIterationValues(i,thetalist,Tsb,Vb,magVbw,magVbv); % prints the iteration values

while err && i < maxiterations
    
    thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
    i = i + 1;
    Tsb = FKinBody(M, Blist, thetalist);
    Vb = se3ToVec(MatrixLog6(TransInv(Tsb) * T));
    magVbw = norm(Vb(1: 3));
    magVbv = norm(Vb(4: 6));
    err = magVbw > eomg || magVbv > ev;
    
    jointVecMatrix((i+1),:) = thetalist';
    
    PrintIterationValues(i,thetalist,Tsb,Vb,magVbw,magVbv);
    
end

csvwrite('Iterates.csv', jointVecMatrix);  % saves the Joint Vector Matrix as comma-separated values

success = ~ err;
end
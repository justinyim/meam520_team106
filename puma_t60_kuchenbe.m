function T60 = puma_t60_kuchenbe(theta1, theta2, theta3, theta4, theta5, theta6)
%
% Given the six joint angles, this function returns the transformation
% matrix for the robot's sixth frame (the end-effector frame). 


%% ROBOT PARAMETERS

% This problem is about the PUMA 260 robot, a 6-DOF manipulator.

% Define the robot's measurements.  These correspond are constant.
a = 13.0; % inches
b =  2.5; % inches
c =  8.0; % inches
d =  2.5; % inches
e =  8.0; % inches
f =  2.5; % inches


%% DH MATRICES

% Calculate the six A matrices using the provided DH function.
A1 = dh_kuchenbe(0,  pi/2,   a, theta1);
A2 = dh_kuchenbe(c,     0,  -b, theta2);
A3 = dh_kuchenbe(0, -pi/2,  -d, theta3);
A4 = dh_kuchenbe(0,  pi/2,   e, theta4);
A5 = dh_kuchenbe(0, -pi/2,   0, theta5);
A6 = dh_kuchenbe(0,     0,   f, theta6);


%% END-EFFECTOR COORDINATE FRAME 

% Save the full transformation in T60.
T60 = A1*A2*A3*A4*A5*A6;

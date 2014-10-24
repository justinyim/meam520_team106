function thetas = team100_puma_ik(x, y, z, phi, theta, psi)
%% team100_puma_ik.m
%
% Calculates the full inverse kinematics for the PUMA 260.
%
% This Matlab file provides the starter code for the PUMA 260 inverse
% kinematics function of Project 1 in MEAM 520 at the University of
% Pennsylvania.  The original was written by Professor Katherine J.
% Kuchenbecker. Students will work in teams modify this code to create
% their own script. Post questions on the class's Piazza forum. 
%
% The first three input arguments (x, y, z) are the desired coordinates of
% the PUMA's end-effector tip in inches, specified in the base frame.  The
% origin of the base frame is where the first joint axis (waist) intersects
% the table. The z0 axis points up, and the x0 axis points out away from
% the robot, perpendicular to the front edge of the table.  These arguments
% are mandatory.
%
%     x: x-coordinate of the origin of frame 6 in frame 0, in inches
%     y: y-coordinate of the origin of frame 6 in frame 0, in inches
%     z: z-coordinate of the origin of frame 6 in frame 0, in inches
%
% The fourth through sixth input arguments (phi, theta, psi) represent the
% desired orientation of the PUMA's end-effector in the base frame using
% ZYZ Euler angles in radians.  These arguments are mandatory.
%
%     phi: first ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%     theta: second ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%     psi: third ZYZ Euler angle to represent orientation of frame 6 in frame 0, in radians
%
% The output (thetas) is a matrix that contains the joint angles needed to
% place the PUMA's end-effector at the desired position and in the desired
% orientation. The first row is theta1, the second row is theta2, etc., so
% it has six rows.  The number of columns is the number of inverse
% kinematics solutions that were found; each column should contain a set
% of joint angles that place the robot's end-effector in the desired pose.
% These joint angles are specified in radians according to the
% order, zeroing, and sign conventions described in the documentation.  If
% this function cannot find a solution to the inverse kinematics problem,
% it will pass back NaN (not a number) for all of the thetas.


%% CHECK INPUTS

% Look at the number of arguments the user has passed in to make sure this
% function is being called correctly.
if (nargin < 6)
    error('Not enough input arguments.  You need six.')
elseif (nargin == 6)
    % This the correct way to call this function, so we don't need to do
    % anything special.
elseif (nargin > 6)
    error('Too many input arguments.  You need six.')
end


%% CALCULATE INVERSE KINEMATICS SOLUTION(S)

% For now, just set the solution to the home position.
th1 = 0;
th2 = 0;
th3 = 0;
th4 = 0;
th5 = -pi/2;
th6 = 0;

% So that we can see the robot move, change th1 to pertain to the
% y-coordinate that was passed in. 
th1 = (y - 5) / 10;

% And change th2 to pertain to the z-coordinate that was passed in.
th2 = (z - 21) / 10;


%% FORMAT OUTPUT

% Put all of the thetas into a column vector to return.
thetas = [th1; th2; th3; th4; th5; th6];

% By the very end, each column of thetas should hold a set of joint angles
% in radians that will put the PUMA's end-effector in the desired
% configuration.  If the desired configuration is not reachable, set all of
% the joint angles to NaN.
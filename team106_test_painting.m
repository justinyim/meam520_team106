%% team106_test_painting.m
%
% This Matlab script is part of the starter code for the simulated light
% painting part of Project 1 in MEAM 520 at the University of Pennsylvania.
% It tests a team's inverse kinematics code for their light painting.


%% SETUP

% Clear all variables from the workspace.
clear all

% Clear the console, so you can more easily find any errors that may occur.
clc

% Set whether to animate the robot's movement and how much to slow it down.
pause on;  % Set this to off if you don't want to watch the animation.
GraphingTimeDelay = 0.05; % The length of time that Matlab should pause between positions when graphing, if at all, in seconds.


%% CREATE JOINT ANGLE SEQUENCE

% To test the positions and orientations from the team's light painting, we
% call the team's light painting code.

% Initialize the function that calculates positions, orientations, and
% colors by calling it once with no arguments.  It returns the duration of
% the light painting in seconds.
duration = team100_get_poc();

% Create time vector.
tstep = 0.1; % s
t = 0:tstep:duration;

% Step through the time vector, filling the histories by calling
% the function that returns positions, orientations, and colors.
for i = 1:length(t)
    [~, ox_history(i), oy_history(i), oz_history(i), phi_history(i), ...
        theta_history(i), psi_history(i), r_history(i), g_history(i), ...
        b_history(i)] = team100_get_poc(t(i));
end


%% TEST

% Notify the user that we're starting the test.
disp('Starting the test.')

% Show a message to explain how to cancel the test and graphing.
disp('Click in this window and press control-c to stop the code.')

% Plot the robot once in the home position to get the plot handles.
figure(1); clf
h = plot_puma_kuchenbe(0,0,0,0,0,0,0,0,0,0,-pi/2,0,0,0,0);

% Step through the ox_history vector to test the inverse kinematics.
for i = 1:length(ox_history)
    
    % Pull the current values of ox, oy, and oz from their histories. 
    ox = ox_history(i);
    oy = oy_history(i);
    oz = oz_history(i);
    
    % Pull the current values of phi, theta, and psi from their histories. 
    phi = phi_history(i);
    theta = theta_history(i);
    psi = psi_history(i);

    % Pull the current values of red, green, and blue from their histories.
    r = r_history(i);
    g = g_history(i);
    b = b_history(i);
        
    % Send the desired pose into the inverse kinematics to obtain the joint
    % angles that will place the PUMA's end-effector at this position and
    % orientation relative to frame 0.
    allthetas = team100_puma_ik(ox, oy, oz, phi, theta, psi);
    
    if (i == 1)
        % Pass in the home position as the current position.
        thetas_history(i,:) = team100_choose_solution(allthetas, [0 0 0 0 -pi/2 0]')';
    else        
        % Choose the solution to show.
        thetas_history(i,:) = team100_choose_solution(allthetas, thetas_history(i-1,:))';
    end

    % Plot the robot at this IK solution with the specified color.        
    plot_puma_kuchenbe(ox,oy,oz,phi,theta,psi,thetas_history(i,1),...
        thetas_history(i,2),thetas_history(i,3),thetas_history(i,4),...
        thetas_history(i,5),thetas_history(i,6),r,g,b,h);

    % Set the title to show the viewer which pose and result this is.
    title(['Pose ' num2str(i)])

    % Pause for a short duration so that the viewer can watch animation evolve over time.
    pause(GraphingTimeDelay)
    
end


%% FINISHED

% Tell the user.
disp('Done with the test.')

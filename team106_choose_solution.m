function thetasOut = team106_choose_solution(allSolutions, thetasnow)
%% team106_choose_solution.m
%
% Chooses the best inverse kinematics solution from all of the solutions
% passed in.  This decision is based both on the characteristics of the
% PUMA 260 robot and on the robot's current configuration.
%
% This Matlab file provides the starter code for the solution choice
% function of Project 1 in MEAM 520 at the University of Pennsylvania.  The
% original was written by Professor Katherine J. Kuchenbecker. Students
% will work in teams modify this code to create their own script. Post
% questions on the class's Piazza forum.
%
% The first input (allSolutions) is a matrix that contains the joint angles
% needed to place the PUMA's end-effector at the desired position and in
% the desired orientation. The first row is theta1, the second row is
% theta2, etc., so it has six rows.  The number of columns is the number of
% inverse kinematics solutions that were found; each column should contain
% a set of joint angles that place the robot's end-effector in the desired
% pose. These joint angles are specified in radians according to the 
% order, zeroing, and sign conventions described in the documentation.  If
% the IK function could not find a solution to the inverse kinematics problem,
% it will pass back NaN (not a number) for all of the thetas.
%
%    allSolutions: IK solutions for all six joints, in radians
%
% The second input is a vector of the PUMA robot's current joint angles
% (thetasnow) in radians.  This information enables this function to
% choose the solution that is closest to the robot's current pose. 
%
%     thetasnow: current values of theta1 through theta6, in radians
%
% Please change the name of this file and the function declaration on the
% first line above to include your team number rather than 100.


%% Puma 260 Limit Parameters
% note that theta4 and theta6 have greater than 360 degree range
limits = [-180 110;...
          -75 240;...
          -235 60;...
          -580 40;...
          -120 110;...
          -215 295];
limits = limits.*(pi/180);


%%
% first wrap solutions with less than 360 range into range
for ii = 1:size(allSolutions,2)
    for jj = 1:6
        while (allSolutions(jj, ii) < limits(jj,1))
            allSolutions(jj, ii) = allSolutions(jj, ii) + 2*pi;
        end
        while (allSolutions(jj, ii) > limits(jj, 2))
            allSolutions(jj, ii) = allSolutions(jj, ii) - 2*pi;
        end
    end
end

% now add solutions due to extra range for theta4/6
temp1 = allSolutions;
temp2 = allSolutions;
temp3 = allSolutions;
temp4 = allSolutions;

temp1(4,:) = temp1(4,:) + 2*pi;
temp2(4,:) = temp2(4,:) - 2*pi;
temp1(6,:) = temp1(6,:) + 2*pi;
temp2(6,:) = temp2(6,:) - 2*pi;

% NaN solutions outside of limits
thetas = team106_sanitize_outputs([allSolutions temp1 temp2 temp3 temp4]);

% remove NaN columns
thetas = thetas(:,all(~isnan(thetas)));


%% Now evaluate distance metrics
distanceMetric = @team106_distanceMetric_l2norm;
% distanceMetric = @team106_distanceMetric_linfinitynorm;

metricOut = zeros(size(thetas,2));
for ii = 1:length(metricOut)
    metricOut(ii) = distanceMetric(thetas(:,ii), thetasnow);
end


%% Finally choose the solution and return it
n = metricOut == min(metricOut);
thetasOut = thetas(:,n);


%%
% You will need to update this function so it chooses intelligently from
% the provided solutions to choose the best one.
%
% There are several reasons why one solution might be better than the
% others, including how close it is to the robot's current configuration
% and whether it violates or obeys the robot's joint limits.
%
% Note that some of the PUMA's joints wrap around, while your solutions
% probably include angles only from -pi to pi or 0 to 2*pi.  If a joint
% wraps around, there can be multiple ways for the robot to achieve the
% same IK solution (the given angle as well as the given angle plus or
% minus 2*pi*n). Be careful about this point.

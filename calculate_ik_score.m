%% calculate_ik_score.m
% Tests the full inverse kinematics for the PUMA 260 for one team,
% calculating a score that reflects its overall performance.
% Written by Katherine J. Kuchenbecker for MEAM 520 at the University of
% Pennsylvania.  Originally drafted in Fall 2013 and updated in Fall 2014.


%% Clear

% Clear the workspace and home the console to make it easier to find
% outputs and errors. 
clear all
home


%% Set IK function name

% Define the name of your the inverse kinematics function you want to test.
% The @ symbol in front of the function name creates a function handle, so
% that we can call the function indirectly.  We do this so that you can
% quickly change the IK function that you want to test, without having to
% change it in many places down below in the code.
puma_ik = @team106_puma_ik;
disp(['Testing function ' func2str(puma_ik) '.m'])


%% Set up empty matrices

% Set the number of tests we want to run.
nTests = 2000;

% Initialize history vectors with all zeros.
ox_history = zeros(nTests,1);
oy_history = zeros(nTests,1);
oz_history = zeros(nTests,1);
phi_history = zeros(nTests,1);
theta_history = zeros(nTests,1);
psi_history = zeros(nTests,1);

% Set the first test to be the home position.
T60home = puma_t60_kuchenbe(0,0,0,0,-pi/2,0);
ox_history(1) = T60home(1,4);
oy_history(1) = T60home(2,4);
oz_history(1) = T60home(3,4);
phi_history(1) = 0;
theta_history(1) = pi/2;
psi_history(1) = 0;

% Set number of manually specified tests from above.
nManualTests = 1; % Set this to zero if you don't want to test the home position.


%% Set up random tests

% Set the number of random poses to generate.
nRandomTests = nTests - nManualTests;

% Reset the random number generator to its default startup settings, so
% that rand produces the same random numbers every time.
rng('default')

% Choose random values for all joint angles.
% rand fills the matrix by columns, so we make those the test cases and
% transpose it so that the first test is always the same, as is the second,
% etc.
thetasr = rand(6,nRandomTests)' * 2 * pi; % radians
th1r = thetasr(:,1); % radians
th2r = thetasr(:,2); % radians
th3r = thetasr(:,3); % radians
th4r = thetasr(:,4); % radians
th5r = thetasr(:,5); % radians
th6r = thetasr(:,6); % radians


%% Convert random joint angles
for i = 1:nRandomTests

    % Push the ith set of random joint angles through the forward
    % kinematics to get the resulting transformation matrix.
    T60rand = puma_t60_kuchenbe(th1r(i),th2r(i),th3r(i),th4r(i),th5r(i),th6r(i));

    % Put the resulting x, y, and z tip positions into the history vectors.
    ox_history(i+nManualTests) = T60rand(1,4);
    oy_history(i+nManualTests) = T60rand(2,4);
    oz_history(i+nManualTests) = T60rand(3,4);

    % Pull out the resulting rotation matrix.
    R = T60rand(1:3,1:3);
    
    % Do inverse Euler angle kinematics - phi around Z, theta around new Y, 
    % and psi around new Z.  
    
    % Calculate one solution for theta.
    costheta = R(3,3);
    sintheta_pos = sqrt(1 - costheta^2);
    theta = atan2(sintheta_pos,costheta);
    
    % Check sine of theta to determine the way to calculate the other two angles.
    if (sin(theta) > 0)
        
        % Calculate phi.
        phi = atan2(R(2,3),R(1,3));
        
        % Calculate psi.
        psi = atan2(R(3,2),-R(3,1));
        
    else

        % Calculate phi.
        phi = atan2(-R(2,3),-R(1,3));
        
        % Calculate psi.
        psi = atan2(-R(3,2),R(3,1));
        
    end
 
    % Put these values into the history matrix.
    phi_history(i+nManualTests) = phi;
    theta_history(i+nManualTests) = theta;
    psi_history(i+nManualTests) = psi;

end


%% Calculate IK score.
% Calculate a score that reflects the performance of this team's inverse
% kinematics function.

% Fill the score matrix with zeros.  The score matrix has eight columns for
% the eight possible solutions, each of which have six entries for each
% aspect of the score. 
scores = zeros(nTests,8,6);

% Run each test through the IK.
for i = 1:nTests
    
    % Show the user that we are making progress.
    if (~mod(i,200))
        disp(i)
    end
    
    % Run the puma_ik inverse kinematics function on the ith pose.
    thetas = puma_ik(ox_history(i), oy_history(i), oz_history(i), phi_history(i), theta_history(i), psi_history(i));

    % Get the size of the returned matrix.
    s = size(thetas);
    
    % Check number of rows in solution.  It must be 6.
    if (s(1) ~= 6)

        % There are not six rows, so give this test a -1 score across the board.
        scores(i,:,:) = -1;
        disp('Trouble.')

    else
        
        % Check number of columns.  We expect from 1 to 8.
        if (s(2) > 8)
            
            % There are more than eight solutions, so give this test a
            % -2 score across the board.
            scores(i,:,:) = -2;
            disp('Double trouble.')
        end
        
        % Remove repeated solutions, if there are any.
        thetas = (unique(thetas','rows','stable'))';
        
        % Get new size of thetas matrix.
        s2 = size(thetas);
        
        % Calculate sine and cosine of the three Euler angles.
        sph = sin(phi_history(i));
        cph = cos(phi_history(i));
        sth = sin(theta_history(i));
        cth = cos(theta_history(i));
        sps = sin(psi_history(i));
        cps = cos(psi_history(i));
            
        % Calculate desired rotation matrix from Euler angles.
        Rdes = [cph -sph 0 ; sph cph 0 ; 0 0 1] * ...
               [cth 0 sth  ; 0 1 0     ; -sth 0 cth ] * ...
               [cps -sps 0 ; sps cps 0 ; 0 0 1];
        
        % Go through each returned solution to determine whether it's
        % correct.
        for j = 1:s2(2)
            
            % Check for NaN values.
            if(any(isnan(thetas(:,j))))
                % All situations tested are reachable, so give this solution
                % a -3 score.
                scores(i,j,:) = -3;
            else
                
                % Push the returned joint angles though the forward kinematics
                % of the robot.
                T60 = puma_t60_kuchenbe(thetas(1,j), thetas(2,j), thetas(3,j), ...
                    thetas(4,j), thetas(5,j), thetas(6,j));
                
                % Calculate x, y, and z position scores as one minus the
                % absolute distance error in inches in that direction.  A
                % correct solution will have zero error, so the score will
                % be 1.  Worse solutions have more negative scores.
                scores(i,j,1) = 1 - abs(T60(1,4) - ox_history(i));
                scores(i,j,2) = 1 - abs(T60(2,4) - oy_history(i));
                scores(i,j,3) = 1 - abs(T60(3,4) - oz_history(i));
                
                % Calculate x, y, and z orientation scores as -9 plus ten
                % times the dot product between the desired final frame
                % unit vector and athe actual final frame unit vector.  A
                % correct solution will have a dot product of one, so the
                % score will be 1.  Worse solutions will have more negative
                % scores.
                scores(i,j,4) = -9+10*dot(Rdes(:,1),T60(1:3,1));
                scores(i,j,5) = -9+10*dot(Rdes(:,2),T60(1:3,2));
                scores(i,j,6) = -9+10*dot(Rdes(:,3),T60(1:3,3));
            end
        end
    
    end
end


%% Calculate scores

% Calculate and display some summary statistics.
disp('What percentage of each of the eight solutions did your code find?')
solScores = squeeze(sum(sum((scores>0).*scores,1),3) * 100 / (nTests * 6))'
disp('What percentage correct did your code get in each category?')
disp('  The first three categories are x, y, and z position of the tip.')
disp('  The next three are the orientation of the x, y, and z axes of the end-effector frame.')
catScores = squeeze(sum(sum((scores>0).*scores)) * 100 / (nTests * 8))'

% Remove the semicolon at the end of this line to see the detailed matrix of scores.
solCatScores = squeeze(sum((scores>0).*scores) * 100 / nTests);

% Calculate total score, which goes from 0 to 100.
disp('What is your code''s total score (out of 100)?')
totalScore = sum(sum(sum((scores>0).*scores))) * 100 / (nTests * 8 * 6)

% Give some compliments.
if (totalScore >= 50)
    disp('Woohoo!  It looks like you found at least four correct solutions.')
end
if (totalScore > 99.8)
    disp('Yippeee!!!   You found all of the IK solutions for the PUMA.  Good work!')
end
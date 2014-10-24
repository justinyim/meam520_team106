thetasR = [-0.4929    3.3805   -0.4929    3.3805   -0.4929    3.3805   -0.4929    3.3805;...
   -0.7124    3.8540    0.5062    2.6354   -0.7124    3.8540    0.5062    2.6354;...
   -0.3522    3.4938    3.4938   -0.3522   -0.3522    3.4938    3.4938   -0.3522;...
    5.1962    1.8670    5.2063    2.5182    2.0546    5.0086    2.0647    5.6598;...
    1.2459    0.6521    1.8793    1.4627   -1.2459   -0.6521   -1.8793   -1.4627;...
    2.9733    2.6795    1.9138    1.5528    6.1149    5.8211    5.0554    4.6944];

allSolutions = thetasR;

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
    metricOut(ii) = distanceMetric(thetas(:,ii), thetasNow);
end




function [ thetas ] = team106_sanitize_outputs( thetas )
% define limits
limits = [-180 110;...
          -75 240;...
          -235 60;...
          -580 40;...
          -120 110;...
          -215 295];
limits = limits.*(pi/180);

for j = 1:size(thetas, 2)
    unreachable = false;
    for i = 1:size(thetas, 1) 
        if (thetas(i, j) < limits(i, 1) || thetas(i, j) > limits(i, 2))
            unreachable = true;
            break;
        end
    end
    if unreachable
        thetas(:,j) = NaN .* ones(size(thetas(:,j)));
    end
end


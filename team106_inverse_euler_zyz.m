function [ phi, theta, psi ] = team106_inverse_euler_zyz( R, soln )
% Radians
    if (abs(R(3,3) - 1) < eps)
        theta = 0;
        phi = 0;
        psi = atan2(R(2,1), R(1,1));
        
    elseif ((R(3,3) + 1) < eps)
        theta = pi;
        phi = 0;
        psi = -atan2(-R(2,1), -R(1,1));
        
    elseif (soln == 1)
        theta = atan2(sqrt(1-R(3,3)^2), R(3,3));
        phi = atan2(R(2,3), R(1,3));
        psi = atan2(R(3,2), -R(3,1));

    elseif (soln == 2)
        theta = atan2(-sqrt(1-R(3,3)^2), R(3,3));
        phi = atan2(-R(2,3), -R(1,3));
        psi = atan2(-R(3,2), R(3,1));
        
    else
        phi = NaN;
        theta = NaN;
        psi = NaN;
        
    end

end


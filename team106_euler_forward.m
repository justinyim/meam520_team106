function [ R ] = team_106_euler_forward( phi, theta, psi )
% Generates orientation matrix R from ZYZ Euler angles phi, theta, and psi
    Rphi = team106_rotz(phi);
    Rtheta = team106_roty(theta);
    Rpsi = team106_rotz(psi);

    R = Rphi*Rtheta*Rpsi;
end


function [thetas] = team106_inverse_position(x,y,z)

%Initialize output
thetas = nan(3,4);

%Robot parameters
a = 13;
b = 2.5;
c = 8;
d = 2.5;
e = 8;
%f = 2.5;

%Theta 1
phi1 = atan2(y,x);
r = sqrt(x^2 + y^2 - (b+d)^2);
alpha1 = atan2(b+d,r);

thetas(1,1) = phi1-alpha1; %left, down
thetas(1,2) = phi1+alpha1+pi; %right, down
thetas(1,3) = thetas(1,1); %left, up
thetas(1,4) = thetas(1,2); %right, up

%Theta 2 and Theta 3
rho = sqrt(r^2 + (z-a)^2);
gamma3 = acos((c^2 + e^2 - rho^2)/(2*c*e));
alpha2 = acos((rho^2 + c^2 - e^2)/(2*rho*c));
epsilon2 = atan2(z-a,r);

thetas(2,1) = epsilon2 - alpha2; %left, down
thetas(2,2) = pi - thetas(2,1); %right, down
thetas(2,3) = epsilon2 + alpha2; %left, up
thetas(2,4) = pi - thetas(2,3); %right, up

thetas(3,1) = pi/2 - gamma3; %left, down
thetas(3,2) = pi/2 + gamma3; %right, down
thetas(3,3) = thetas(3,2); %left, up
thetas(3,4) = thetas(3,1); %right, up









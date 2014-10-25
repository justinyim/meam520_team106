clear all;
load traversal
load coords

figure(1);
hold on;

%%
t = 1;

rotMat1 = [ cos(t)  sin(t)  0  0;
          -sin(t)  cos(t)  0  0; % XY rotation
             0         0       1  0;
             0         0       0  1];

rotMat2 = [1  0    0        0;
           0  1    0        0;    % ZW rotation
           0  0  cos(t)  -sin(t);
           0  0  sin(t)   cos(t)];  

points = coords([traversal(1,2);traversal(:,3)],2:end)';
rot_points = rotMat1*rotMat2*points;
points3 = 2*rot_points(1:3,:)./abs(3-ones(3,1)*rot_points(4,:));
plot3(points3(1,:),points3(2,:),points3(3,:),'b-')
axis equal
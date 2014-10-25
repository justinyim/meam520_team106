clear all;
load traversal
load coords


%%
points = coords([traversal(1,2);traversal(:,3)],2:end)';
n = size(points,2);

t = 1;

rotMat1 = [ cos(t)  sin(t)  0  0;
          -sin(t)  cos(t)  0  0; % XY rotation
             0         0       1  0;
             0         0       0  1];

rotMat2 = [1  0    0        0;
           0  1    0        0;    % ZW rotation
           0  0  cos(t)  -sin(t);
           0  0  sin(t)   cos(t)];  


rot_points = rotMat1*rotMat2*points;
unit_points3 = 2*rot_points(1:3,:)./abs(3-ones(3,1)*rot_points(4,:));

points3 = 2*unit_points3 + [13; 0; 15]*ones(1,n);

figure(1);
clf
plot3(points3(1,:),points3(2,:),points3(3,:),'b-')
axis equal


painting = [points3', zeros(n,1), pi/2*ones(n,1), zeros(n,1), zeros(n,1), ones(n,1), zeros(n,1), zeros(n,1)];
save('team106.mat','painting');

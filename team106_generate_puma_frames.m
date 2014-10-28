clear all;
load traversal
load coords

h_avi = VideoWriter('prelim_vid');
h_avi.FrameRate = 20;
h_avi.open();


%%
kk = 1;
for t = 0:(pi/40):pi/2
points = coords([traversal(1,2);traversal(:,3)],2:end)';
n = size(points,2);

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

unit_points3 = team106_roty(pi/6)*team106_rotz(pi/8)*unit_points3;

points3 = 2.1*unit_points3 + [12.5; 3; 20]*ones(1,n);

figure(2);
clf
plot3(points3(1,:),points3(2,:),points3(3,:),'b-')
axis equal


painting = [points3', zeros(n,1), pi/2*ones(n,1), zeros(n,1), zeros(n,1), ones(n,1), zeros(n,1), zeros(n,1)];
save(['team106_' sprintf('%02d', kk) '.mat'],'painting');
save('team106.mat','painting');
kk = kk + 1;

team106_test_painting;
% team106_puma_paint;

pic = getframe(gcf);
h_avi.writeVideo(pic);

end

% Finish video
h_avi.close();
clear h_avi;
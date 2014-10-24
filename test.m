clear all;
load traversal
load coords

figure(1);
hold on;

%%

for ii = 1:size(traversal,1)
    n1 = traversal(ii,2);
    n2 = traversal(ii,3);
    
    p1 = coords(n1, 2:end);
    p2 = coords(n2, 2:end);
    
    p1_3 = 5*p1(1:3)./abs(3-ones(1,3)*p1(4));
    p2_3 = 5*p2(1:3)./abs(3-ones(1,3)*p2(4));
    
    edge = [p1_3; p2_3];
    
    plot3(edge(:,1), edge(:,2), edge(:,3), 'b-');
end
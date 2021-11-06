function [xn] = animation(t, z, x0)
global l;
global m;
global I;
global S;
global g;
global theta;
%define Support leg position
starting_point = x0;

for i = 1: length(t)
support_x = -l*sin(z(i,1))+x0(1);
support_y = l*cos(z(i,1));    
swing_x  =  -l*sin(z(i,1))-l*sin(z(i,1)+z(i,2))+x0(1);
swing_y =  l*cos(z(i,1))+l*cos(z(i,1)+z(i,2));

%Define the hip and the swing position
hip_position = [support_x support_y];
swing_tip = [swing_x swing_y];

%define the 3 points that make up the 2 legs
points = [starting_point; hip_position; swing_tip];

hold off;
plot(points(:,1),points(:,2), 'k', 'LineWidth', 4);
%plotting some points at the tips and hip
hold on;
plot(x0(1), x0(2), 'r*');
hold on;
plot(support_x, support_y, 'g*');
hold on;
plot(swing_x, swing_y, 'r*');
axis([-3 10 0 2]);
hold on;
drawnow;
end

xn= swing_tip;
end

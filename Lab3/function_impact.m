function [A1, JR] = function_impact( q1, q2)
%My reaction force

%% Constants
l = 0.8; %m
m = 2; %kg
I = 0.1; %kg*m^2
S = 0.5; %m
g = 9.8; %m*s^-2

%% Matrices
a11 = 2*m;
a12 = 0;
a13 = m*S*cos(q1) -m*S*cos(q1 + q2);
a14 = -m*S*cos(q1 + q2);
a21 = 0;
a22 = 2*m;
a23 = m*S*cos(q1) -m*S*sin(q1+q2);
a24 = -m*S*sin(q1+q2);
a31 = m*S*cos(q1) -m*S*cos(q1 + q2);
a32 = m*S*cos(q1) -m*S*sin(q1+q2);
a33 = 2*m*S^2 +2*I;
a34 = m*S^2 +I;
a41 = -m*S*cos(q1 + q2);
a42 = -m*S*sin(q1 + q2);
a43 = m*S^2 +I;
a44 = m*S^2 +I;


A1 = [a11 a12 a13 a14; a21 a22 a23 a24; a31 a32 a33 a34; a41 a42 a43 a44];

JR = [1  0  -l*cos(q1+q2) -l*cos(q1+q2); 0 1 -l*sin(q1+q2) -l*sin(q1+q2)];
end


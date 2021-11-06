function [F] = function_reactionforce(q1,q2,q1d,q2d,q1dd,q2dd,theta)
%My reaction force under the formF= 2mxddf -smg
%% Constants

l = 0.8; %m
m = 2; %kg
I = 0.1; %kg*m^2
S = 0.5; %m
g = 9.8; %m*s^-2

%% Model
%gravity, previously defined 
G = [g*sin(theta); -g*cos(theta)];

%acceleration
xdd = l*sin(q1)*q1d^2 -l*cos(q1)*q1dd +0.5*S*cos(q1)*q1dd -0.5*S*sin(q1)*q1d^2 +0.5*S*sin(q1+q2)*(q1d+q2d)^2 -0.5*S*cos(q1+q2)*(q1dd + q2dd);
ydd = -l*cos(q1)*q1d^2 -l*sin(q1)*q1dd +0.5*S*sin(q1)*q1dd +0.5*S*cos(q1)*q1^2 -0.5*S*sin(q1+q2)*(q1dd+q2dd) -0.5*S*cos(q1+q2)*(q1d+q2d)^2;

Xdd=[xdd; ydd];

%reaction force
F = 2*m*Xdd -2*m*G;

end


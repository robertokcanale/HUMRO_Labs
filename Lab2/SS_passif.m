function [zdot] = SS_passif(t,z)
%% Extract state values

global l;
global m;
global I;
global S;
global g;
global theta;
%defines the derivatives of the states, and we can retrieve it from 
%the first lab, as the velocity.
%extract the initial conditions
q1 = z(1);
q2 = z(2);
qd1 = z(3);
qd2 = z(4);
%constants
l = 0.8; %m
S = 0.5; %m
theta =3*pi/180;

%evaluate zdot as the derivative of the displacement and the velocity:
%the velocity and the acceleration respectively
zdot(1) = qd1;
zdot(2) = qd2;

%we however need to extract the acceleration, then go back to the lab 1 to
%find it
%Aqdd = -H -Q
[A, H, Q] = function_dyn(q1, q2, qd1, qd2, theta);
qdd = inv(A)*(-H-Q); %here i find the dynamic model. 
zdot(3) = qdd(1); 
zdot(4) = qdd(2);
zdot = zdot'; %the function says it must return a columns vector
end
function [A,H, Q] = function_dyn(q1, q2, qd1, qd2, theta)
%My dynamic model under the form A*qdd + H = B*Tau
%% Constants
global l;
global m;
global I;
global S;
global g;
global theta;

%defining this as its saves time
mls =  m*l*S;

%% Matrix computation
%A
a11 = m*(l-S)^2 + m*l^2 + m*S^2 + 2*I + 2*mls*cos(q2);
a12 = m*S^2 + I + mls*cos(q2);
a21 = m*S^2 + I + mls*cos(q2);
a22 = m*S^2 +I;
A = [ a11 a12; a21 a22];
%Q
pot1 = m*g*(l-S)*sin(theta - q1) + m*g*l*sin(theta - q1) + m*g*S*sin(theta - q1 - q2);
pot2 = m*g*S*sin(theta-q1-q2);

Q = [pot1 pot2];
%C
b1=-2*mls*sin(q2);
b2 =0;
B = [b1; b2];
%C
c11 = 0;
c12 = -mls*sin(q2);
c21 = mls*sin(q2);
c22= 0;
C = [c11 c12; c21 c22];

%my angles
qd = qd1*qd2;
qd_square = [qd1^2; qd2^2];

H = B*qd +  C*qd_square;

end


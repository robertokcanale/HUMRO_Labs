afunction [C]=resol(Jsolcons0)
%% Constants
global l;
global m;
global I;
global S;
global g;
global theta;
global gt;
global gteq;


T =1; %period
%trajectory from intermediate to final
q1_fin = Jsolcons0(1); %step size
q2_fin = pi-2*q1_fin; %decided arbitrarily
q1_intermediate = Jsolcons0(1);
q2_intermediate = Jsolcons0(2); % look at the drawing on the slides

qd1_fin = Jsolcons0(4);
qd2_fin = Jsolcons0(5); %no speed when the foot lands

ze = [q1_fin; q2_fin; qd1_fin; qd2_fin]';

%% Impact equation
%To use this model, check lab 1, I ONLY CARE ABOUT qdot+
%check lab 1 for the velocity of the hip/support leg
support_xd_minus = -l*cos(ze(1))*ze(3) ;
support_yd_minus = -l*sin(ze(1))*ze(3) ;

%state BEFORE IMPACT
state_minus = [support_xd_minus; support_yd_minus; ze(3); ze(4)];

[A_1, JR] = function_impact( ze(1), ze(2));
A_2 = [A_1 -JR'; JR  zeros(2,2)];

%state AFTER IMPACT
state_plus = inv(A_2)*[A_1; zeros(2,4)]*state_minus;

%from here, we can find the new state and veolcities
z_after =[ze(1); ze(2); state_plus(3); state_plus(4)];

Fx = state_plus(5); %impact forces, check Lab1
Fy = state_plus(6); %impact forces, check Lab1

%Relable the angles
z_new = relabel_equation(z_after);

%% Now find the polynomial trajectory coefficients, from initial to intermediate to final
%update
q1_initial = z_new(1);
q2_initial = z_new(2);
qd1_initial =  z_new(3);
qd2_initial =  z_new(4);

traj1_points = [q1_initial; qd1_initial; q1_intermediate;  q1_fin; qd1_fin];
traj2_points = [q2_initial; qd2_initial; q2_intermediate;  q2_fin; qd2_fin];

%check slides
poly_matrix = [0 0 0 0 1;
               0 0 0 1 0;
               (T/2).^4 (T/2).^3 (T/2).^2 T/2 1;
               T.^4     T.^3     T.^2     T  1;
               4*T.^3 4*T.^2    2*T     1    0];

%find q1 and q2 parameters
A_1 =  inv(poly_matrix)*traj1_points; %these are 5 by1
A_2 =  inv(poly_matrix)*traj2_points; %5x1
a01 = A_1(5);
a02 = A_2(5);
a11 = A_1(4);
a12 = A_2(4);
a21 = A_1(3); 
a22 = A_2(3);
a31 = A_1(2);
a32 = A_2(2);
a41 = A_1(1);
a42 = A_2(1);   

%now actually define the trajectory polynomials
timesteps =100;
t = linspace(0, T, timesteps)';

q1 = a01 + a11 * t + a21 * t.^2 + a31 * t.^3 + a41 * t.^4;
qd1 = a11 + 2* a21 * t + 3 * a31 * t.^2 + 4 * a41 * t.^3;
qdd1 = 2 * a21 + 6 * a31 * t + 12 * a41 * t.^2;

q2 = a02 + a12 * t + a22 * t.^2 + a32 * t.^3 + a42 * t.^4;
qd2 = a12 + 2 * a22 * t + 3 * a32 * t.^2 + 4 * a42 * t.^3;
qdd2 = 2 * a22 + 6 * a32 * t + 12 * a42 * t.^2;

% Step length
L_step = 2*l*sin(q1_fin);

%% Inverse dynamic system 

C = 0;
for i = 1 : timesteps
[A,H,Q] = function_dyn(q1(i), q2(i), qd1(i), qd2(i), theta);

qdd = [qdd1(i) ; qdd2(i)];

tau(:,i) = A*qdd + H + Q;

%% Reaction Forces

F = function_reactionforce(q1(i),q2(i),qd1(i),qd2(i),qdd1(i),qdd2(i),theta);

Rx(i) = F(1);
Ry(i) = F(2);

%% Find constraint C @ each iteration

C = C + (1/L_step)*(tau(1,i)^2 + tau(2, i)^2)*(T/timesteps);
end 

%% Setting the constraints 
gt = [abs(Rx)-0.7*abs(Ry); -Ry; -L_step+0.2*ones(1,timesteps); -Fy*ones(1,timesteps); abs(Fx)*ones(1,timesteps)-0.7*abs(Fy)*ones(1,timesteps); abs(qd1)-3*ones(1,timesteps); abs(qd2)-3*ones(1,timesteps); 
     abs(tau(1))-50*ones(1,timesteps); abs(tau(2))-50*ones(1,timesteps)];

end
%% HUMRO LAB 2
%% Constants
global l;
global m;
global I;
global S;
global g;
global theta;
l = 0.8; %m
m = 2; %kg
I = 0.1; %kg*m^2
S = 0.5; %m
g = 9.8; %m*s^-2
theta =3*pi/180; %rad/s


%% SS phase (one)

%q INITIAL VALUE
q1=0.1860;
q2= 2.7696; 
qd1= -1.4281;
qd2 = 0.3377;


options = odeset('Events', @PEvents);
z0=[q1; q2; qd1; qd2]; % INITIAL STATE

[t, z, te, ze] = ode45(@SS_passif, [0:0.02:10], z0, options);
%my state vector is the position and velocity of the swing leg. 
%z = x, y, xd, yd
%te and ze are the terminal time and state respectively

% I can now animate this

x0 = [0 0]; % initial position
x0 = animation(t, z, x0);

%% Simulation of Several Steps
%my state changes, it now becomes ze, state before impact

%To use this model, check lab 1 
%check lab 1 for the velocity of the hip/support leg
support_xd_minus = -l*cos(ze(1))*ze(3) ;
support_yd_minus = -l*sin(ze(1))*ze(3) ;

%state BEFORE IMPACT
state_minus = [support_xd_minus; support_yd_minus; ze(3); ze(4)];

[A1, JR] = function_impact( ze(1), ze(2));
A2 = [A1 -JR'; JR  zeros(2,2)];

%state AFTER IMPACT
state_plus = inv(A2)*[A1; zeros(2,4)]*state_minus;

%from here, we can find the new state and veolcities
z_after =[ze(1); ze(2); state_plus(3); state_plus(4)];


options = odeset('Events', @PEvents);

[t, z, te, ze] = ode45(@SS_passif, [0:0.02:10], z_new, options);
%%
% Animation of the passive walking
x0 = animation(t, z, x0);

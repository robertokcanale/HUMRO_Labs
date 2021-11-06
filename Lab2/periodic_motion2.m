%% CASE 2
clc;
clear all;
global l;
global m;
global I;
global S;
global g;
global theta;
l = 0.8; %m
m = 2; %kg
I = 0.008; %kg*m^2
S = 0.45; %m
g = 9.8; %m*s^-2
theta =3*pi/180; %rad/s
warning('off')
%% INITIALIZATION
%Starting conditions
q1 = -0.1933;
q2 = pi-2*q1;
qd1 = -2.0262;
qd2 = -0.1253;

x0 = [0 0];

% Initialize cyclic motion

options = odeset('Events', @PEvents);

z0=[q1;q2;qd1;qd2];

[t, z, te, ze] = ode45(@SS_passif, [0:0.02:10], z0, options);

X0 = [ze(1);ze(3);ze(4)];

% Compute X for cyclic motion

options = optimset('display','iter','MaxFunEvals',1000,'TolX', 1.0e-010,'TolFun', 1.0e-006);
X_star = fsolve('periodic_function',X0,options);

% Run cyclic motion
z_cyclic = simulate_motion(10, X_star,t,z);

% Plot resulting phase plane
X_star = [X_star(1),pi-2*X_star(1),X_star(2),X_star(3)];

%% Plotting Phase Planes
figure
subplot(1,2,1);
hold on;
plot(z_cyclic(:,1),z_cyclic(:,3), 'k');
plot(X_star(1), X_star(3), '*r');
title('Phase Plane Joint 1');
xlabel('Dispacement');
ylabel('Velocity');
hold off;

subplot(1,2,2);
hold on;
plot(z_cyclic(:,2),z_cyclic(:,4), 'k');
plot(X_star(2), X_star(4), '*r');
title('Phase Plane Joint 2');
xlabel('Displacement');
ylabel('Velocity');
hold off;

%% Stability Analysis
stab = check_stability(X_star);
if stab == 0
    disp('STABLE');
elseif stab == 1
    disp('NOT STABLE');
end


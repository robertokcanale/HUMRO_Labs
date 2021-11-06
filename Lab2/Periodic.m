 
function [Xnext] = Periodic(X_star)
% Periodic
global l;
global m;
global I;
global S;
global g;
global theta;

q1 = X_star(1);
q2 = pi-2*q1;
qd1 = X_star(2);
qd2 = X_star(3);

    support_xd_minus = -l*cos(X_star(1))*X_star(2);
    support_yd_minus = -l*sin(X_star(1))*X_star(2);

    state_minus = [support_xd_minus;support_yd_minus;X_star(2);X_star(3)];

    % Compute joint velocity after impact 

    [A1, JR] = function_impact( X_star(1), pi-2*X_star(1));
    A2 = [A1, -JR';JR,zeros(2,2)];

    %state AFTER IMPACT
    state_plus = inv(A2)*[A1; zeros(2,4)]*state_minus;
    
    %from here, we can find the new state and veolcities
    z_after =[X_star(1); pi-2*X_star(1); state_plus(3); state_plus(4)];
    
    %After the impact, i need to perform the relabel equation between the new
    %and old support and swing leg
    z_new = relabel_equation(z_after);
    z_new = z_new';

options = odeset('Events', @PEvents);

[t, z, te, ze] = ode45(@SS_passif, [0:0.02:10], z_new, options);

Xnext = [ze(1);ze(3);ze(4)];

end
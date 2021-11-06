function [value, is_terminal_value ,dir] = PEvents (t,z)
% we need to evaluate the value at the tip of the SWING LENG
q1 = z(1);
q2 = z(2);
%constants
global l;
global m;
global I;
global S;
global g;
global theta;

%position of the swing leg tip
x = -l*sin(q1) -l*sin(q1+q2);
y = l*cos(q1) + l*cos(q1+q2);

%setting the values of t y according to the position x 
if x > 0.1
    value = y;
else
    value = 1;
end

        
dir = 0;       
is_terminal_value = 1;

end


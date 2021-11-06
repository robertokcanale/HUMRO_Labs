function [z_new] = relabel_equation(z_old);
z_new(1) =  -pi +z_old(1)+z_old(2);
z_new(2) = -z_old(2);
z_new(3) = +z_old(4) +z_old(3);
z_new(4) = -z_old(4);

end


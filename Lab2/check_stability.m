function [stab] = check_stability(X_star)

stab = 0;

%error velocity
ev = 0.0005;
%error position
ep = 0.00005;
% J as given in the assignment
J1 = (Periodic(X_star + [1;0;0]*ep)-Periodic(X_star - [1;0;0]*ep))/(2*ep);
J2 = (Periodic(X_star + [0;1;0]*ev)-Periodic(X_star - [0;1;0]*ev))/(2*ev);
J3 = (Periodic(X_star + [0;0;1]*ev)-Periodic(X_star - [0;0;1]*ev))/(2*ev);
J = [J1 J2 J3];

% Eigenvalues of the Jacobian
eigenvalues = eig(J); %e = eig(A) returns a column vector containing the eigenvalues of square matrix A.

% eigenvalue is < 1, we set the flag
%All the norm of the eigen-values of J(X*) are strictly less that 1
for i= 1:length(eigenvalues)
    if norm(eigenvalues(i)) >= 1
       stab = 1; %set stability to 1
    end
        
end

end